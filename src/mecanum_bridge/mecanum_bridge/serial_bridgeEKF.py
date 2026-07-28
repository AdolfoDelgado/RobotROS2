#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

import serial
from serial import SerialException


class SerialBridge(Node):
    """
    Puente serial ESP32 <-> ROS 2.

    Tramas recibidas desde el ESP32:

      ODOM,x,y,theta,vx,vy,w,ang1,ang2,ang3,ang4

        x, y                 [m]
        theta               [rad]
        vx, vy              [m/s]
        w                    [rad/s]
        ang1..ang4           [rad]

      IMU,qx,qy,qz,qw,gx,gy,gz,ax,ay,az

        qx, qy, qz, qw      cuaternión de orientación
        gx, gy, gz          [rad/s]
        ax, ay, az          [m/s^2]

    Comandos enviados al ESP32:

      vx,vy,w

        vx, vy              [m/s]
        w                   [rad/s]
    """

    JOINT_NAMES = [
        "WheelLeftAhead_Joint",
        "WheelRightAhead_Joint",
        "WheelRightBack_Joint",
        "WheelLeftBack_Joint",
    ]

    def __init__(self) -> None:
        super().__init__("serial_bridge")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("imu_frame", "IMU_Link")
        self.declare_parameter("publish_odom_tf", False)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_joy")

        port = self.get_parameter("serial_port").value
        baud = int(self.get_parameter("baud_rate").value)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.imu_frame = str(self.get_parameter("imu_frame").value)
        self.publish_odom_tf = bool(
            self.get_parameter("publish_odom_tf").value
        )
        self.cmd_vel_topic = str(
            self.get_parameter("cmd_vel_topic").value
        )

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.01,
            )
            self.ser.reset_input_buffer()
        except SerialException as exc:
            self.get_logger().fatal(
                f"No se pudo abrir el puerto serial {port}: {exc}"
            )
            raise

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        self.odom_pub = self.create_publisher(Odometry, "/odom/raw", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.joint_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # 100 Hz. En cada llamada se procesan todas las líneas disponibles.
        self.timer = self.create_timer(0.01, self.read_serial)

        self.get_logger().info(
            f"Puente serial iniciado en {port} a {baud} baudios; "
            f"escuchando {self.cmd_vel_topic}"
        )
        self.get_logger().info(
            "RX esperado: ODOM,... e IMU,... | Publicando /odom/raw para el EKF"
        )

    @staticmethod
    def quaternion_from_euler(
        roll: float,
        pitch: float,
        yaw: float,
    ) -> tuple[float, float, float, float]:
        """Convierte roll-pitch-yaw en radianes a cuaternión."""
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return qx, qy, qz, qw

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def cmd_vel_callback(self, msg: Twist) -> None:
        data = (
            f"{msg.linear.x:.4f},"
            f"{msg.linear.y:.4f},"
            f"{msg.angular.z:.4f}\n"
        )

        try:
            self.ser.write(data.encode("utf-8"))
        except SerialException as exc:
            self.get_logger().error(
                f"Error enviando /cmd_vel por serial: {exc}"
            )

    def read_serial(self) -> None:
        try:
            # Evita procesar solamente una línea si llegaron ODOM e IMU
            # entre dos llamadas del timer.
            while self.ser.in_waiting > 0:
                raw_line = self.ser.readline()

                try:
                    line = raw_line.decode("utf-8").strip()
                except UnicodeDecodeError:
                    self.get_logger().warning(
                        "Se recibió una línea serial no válida"
                    )
                    continue

                if not line:
                    continue

                self.process_line(line)

        except SerialException as exc:
            self.get_logger().error(f"Error leyendo el puerto serial: {exc}")

    def process_line(self, line: str) -> None:
        fields = [field.strip() for field in line.split(",")]
        message_type = fields[0].upper()

        try:
            if message_type == "ODOM":
                self.publish_odometry(fields)
            elif message_type == "IMU":
                self.publish_imu(fields)
            else:
                self.get_logger().warning(
                    f"Trama desconocida: {line}"
                )
        except (ValueError, IndexError) as exc:
            self.get_logger().warning(
                f"Trama inválida ({exc}): {line}"
            )

    def publish_odometry(self, fields: list[str]) -> None:
        # ODOM + 10 valores = 11 campos
        if len(fields) != 11:
            raise ValueError(
                f"ODOM necesita 11 campos y llegaron {len(fields)}"
            )

        x = float(fields[1])
        y = float(fields[2])
        theta = float(fields[3])
        vx = float(fields[4])
        vy = float(fields[5])
        w = float(fields[6])

        wheel_angles = [
            self.normalize_angle(float(fields[7])),
            self.normalize_angle(float(fields[8])),
            self.normalize_angle(float(fields[9])),
            self.normalize_angle(float(fields[10])),
        ]

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = self.quaternion_from_euler(
            0.0,
            0.0,
            theta,
        )

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = w

        # Valores iniciales razonables; después se pueden ajustar
        # experimentalmente para robot_localization.
        odom.pose.covariance[0] = 0.02       # x
        odom.pose.covariance[7] = 0.02       # y
        odom.pose.covariance[14] = 1.0e6     # z no medido
        odom.pose.covariance[21] = 1.0e6     # roll no medido
        odom.pose.covariance[28] = 1.0e6     # pitch no medido
        odom.pose.covariance[35] = 0.05      # yaw

        odom.twist.covariance[0] = 0.03
        odom.twist.covariance[7] = 0.03
        odom.twist.covariance[14] = 1.0e6
        odom.twist.covariance[21] = 1.0e6
        odom.twist.covariance[28] = 1.0e6
        odom.twist.covariance[35] = 0.08

        self.odom_pub.publish(odom)

        if self.publish_odom_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame

            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = 0.0

            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(transform)

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = self.JOINT_NAMES
        joint_state.position = wheel_angles
        self.joint_pub.publish(joint_state)

    def publish_imu(self, fields: list[str]) -> None:
        # IMU + 10 valores = 11 campos
        if len(fields) != 11:
            raise ValueError(
                f"IMU necesita 11 campos y llegaron {len(fields)}"
            )

        qx = float(fields[1])
        qy = float(fields[2])
        qz = float(fields[3])
        qw = float(fields[4])

        gx = float(fields[5])
        gy = float(fields[6])
        gz = float(fields[7])

        ax = float(fields[8])
        ay = float(fields[9])
        az = float(fields[10])

        # Normaliza el cuaternión por seguridad.
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm < 1.0e-9:
            raise ValueError("cuaternión IMU inválido")

        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.imu_frame

        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw

        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        # Valores iniciales. Deben ajustarse con mediciones reales del BNO055.
        imu.orientation_covariance[0] = 0.02
        imu.orientation_covariance[4] = 0.02
        imu.orientation_covariance[8] = 0.03

        imu.angular_velocity_covariance[0] = 0.02
        imu.angular_velocity_covariance[4] = 0.02
        imu.angular_velocity_covariance[8] = 0.02

        imu.linear_acceleration_covariance[0] = 0.10
        imu.linear_acceleration_covariance[4] = 0.10
        imu.linear_acceleration_covariance[8] = 0.10

        self.imu_pub.publish(imu)

    def destroy_node(self) -> bool:
        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
