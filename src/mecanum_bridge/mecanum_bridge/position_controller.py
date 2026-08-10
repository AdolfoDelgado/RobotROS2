#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class PID:
    """PID sencillo con anti-windup por límite de integral."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        integral_limit: float,
        output_limit: float,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.output_limit = abs(output_limit)

        self.integral = 0.0
        self.previous_error: Optional[float] = None

    def reset(self):
        self.integral = 0.0
        self.previous_error = None

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        # Integral
        self.integral += error * dt
        self.integral = max(
            -self.integral_limit,
            min(self.integral, self.integral_limit),
        )

        # Derivada
        if self.previous_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.previous_error) / dt

        self.previous_error = error

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        return max(-self.output_limit, min(output, self.output_limit))


class PositionController(Node):
    """
    Controlador PID de posición para un robot mecanum.

    El objetivo se define como desplazamiento RELATIVO a la posición
    que tenía el robot cuando arrancó el nodo:

        target_x     -> desplazamiento en X [m]
        target_y     -> desplazamiento en Y [m]
        target_theta -> giro [rad]

    El nodo:
        1. Lee la pose actual desde /odometry/filtered.
        2. Calcula el error de posición/orientación.
        3. Aplica PID independiente a X, Y y theta.
        4. Convierte vx/vy del marco global al marco del robot.
        5. Publica Twist en /cmd_vel_joy, que es el tópico que
           escucha tu serial_bridge.
    """

    def __init__(self):
        super().__init__("position_controller")

        # ------------------------------------------------------------
        # Tópicos
        # ------------------------------------------------------------
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_joy")

        # ------------------------------------------------------------
        # Objetivo relativo al punto inicial
        # ------------------------------------------------------------
        self.declare_parameter("target_x", 0.0)       # metros
        self.declare_parameter("target_y", 0.0)       # metros
        self.declare_parameter("target_theta", 0.0)   # radianes

        # ------------------------------------------------------------
        # Ganancias PID
        # ------------------------------------------------------------
        self.declare_parameter("kp_x", 0.8)
        self.declare_parameter("ki_x", 0.0)
        self.declare_parameter("kd_x", 0.0)

        self.declare_parameter("kp_y", 0.8)
        self.declare_parameter("ki_y", 0.0)
        self.declare_parameter("kd_y", 0.0)

        self.declare_parameter("kp_theta", 1.5)
        self.declare_parameter("ki_theta", 0.0)
        self.declare_parameter("kd_theta", 0.0)

        # ------------------------------------------------------------
        # Límites
        # ------------------------------------------------------------
        self.declare_parameter("max_vx", 0.30)        # m/s
        self.declare_parameter("max_vy", 0.30)        # m/s
        self.declare_parameter("max_w", 0.80)         # rad/s
        self.declare_parameter("min_vx", 0.15)        # m/s
        self.declare_parameter("min_vy", 0.15)        # m/s
        self.declare_parameter("min_w", 0.10)         # rad/s

        self.declare_parameter("integral_limit_xy", 0.5)
        self.declare_parameter("integral_limit_theta", 1.0)

        # ------------------------------------------------------------
        # Tolerancias
        # ------------------------------------------------------------
        self.declare_parameter("position_tolerance", 0.02)   # m
        self.declare_parameter("theta_tolerance", 0.03)      # rad

        # Tiempo máximo sin recibir odometría
        self.declare_parameter("odom_timeout", 0.5)

        # Frecuencia del controlador
        self.declare_parameter("control_frequency", 50.0)

        # ------------------------------------------------------------
        # Parámetros
        # ------------------------------------------------------------
        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self.target_x = float(self.get_parameter("target_x").value)
        self.target_y = float(self.get_parameter("target_y").value)
        self.target_theta = float(
            self.get_parameter("target_theta").value
        )

        self.max_vx = abs(float(self.get_parameter("max_vx").value))
        self.max_vy = abs(float(self.get_parameter("max_vy").value))
        self.max_w = abs(float(self.get_parameter("max_w").value))
        self.min_vx = min(abs(float(self.get_parameter("min_vx").value)), self.max_vx)
        self.min_vy = min(abs(float(self.get_parameter("min_vy").value)), self.max_vy)
        self.min_w = min(abs(float(self.get_parameter("min_w").value)), self.max_w)

        self.position_tolerance = float(
            self.get_parameter("position_tolerance").value
        )
        self.theta_tolerance = float(
            self.get_parameter("theta_tolerance").value
        )

        self.odom_timeout = float(
            self.get_parameter("odom_timeout").value
        )

        frequency = float(
            self.get_parameter("control_frequency").value
        )

        # ------------------------------------------------------------
        # PID
        # ------------------------------------------------------------
        self.pid_x = PID(
            float(self.get_parameter("kp_x").value),
            float(self.get_parameter("ki_x").value),
            float(self.get_parameter("kd_x").value),
            float(self.get_parameter("integral_limit_xy").value),
            self.max_vx,
        )

        self.pid_y = PID(
            float(self.get_parameter("kp_y").value),
            float(self.get_parameter("ki_y").value),
            float(self.get_parameter("kd_y").value),
            float(self.get_parameter("integral_limit_xy").value),
            self.max_vy,
        )

        self.pid_theta = PID(
            float(self.get_parameter("kp_theta").value),
            float(self.get_parameter("ki_theta").value),
            float(self.get_parameter("kd_theta").value),
            float(self.get_parameter("integral_limit_theta").value),
            self.max_w,
        )

        # ------------------------------------------------------------
        # Estado
        # ------------------------------------------------------------
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.start_x: Optional[float] = None
        self.start_y: Optional[float] = None
        self.start_theta: Optional[float] = None

        self.last_odom_time = None
        self.last_control_time = self.get_clock().now()

        self.goal_reached = False
        self.have_odom = False

        # ------------------------------------------------------------
        # ROS
        # ------------------------------------------------------------
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10,
        )

        period = 1.0 / max(frequency, 1.0)

        self.timer = self.create_timer(
            period,
            self.control_loop,
        )

        self.get_logger().info(
            "Controlador PID de posición iniciado."
        )
        self.get_logger().info(
            f"Odometría: {odom_topic}"
        )
        self.get_logger().info(
            f"Comando: {cmd_vel_topic}"
        )
        self.get_logger().info(
            "Objetivo relativo: "
            f"X={self.target_x:.3f} m, "
            f"Y={self.target_y:.3f} m, "
            f"Theta={self.target_theta:.3f} rad"
        )

    # ================================================================
    # Utilidades
    # ================================================================

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Lleva un ángulo al intervalo [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def apply_min_speed(velocity: float, min_speed: float, max_speed: float) -> float:
        """Aplica una velocidad mínima conservando el signo; cero permanece en cero."""
        if abs(velocity) < 1.0e-6:
            return 0.0
        magnitude = min(max(abs(velocity), min_speed), max_speed)
        return math.copysign(magnitude, velocity)

    def publish_zero(self):
        """Detiene completamente el robot."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.cmd_pub.publish(msg)

    # ================================================================
    # Odometría
    # ================================================================

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        # Yaw a partir del cuaternión
        sin_yaw = 2.0 * (
            q.w * q.z + q.x * q.y
        )
        cos_yaw = 1.0 - 2.0 * (
            q.y * q.y + q.z * q.z
        )

        self.current_theta = math.atan2(
            sin_yaw,
            cos_yaw,
        )

        self.last_odom_time = self.get_clock().now()
        self.have_odom = True

        # ------------------------------------------------------------
        # Guardamos la posición inicial.
        # ------------------------------------------------------------
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_theta = self.current_theta

            self.get_logger().info(
                "Posición inicial capturada:"
            )
            self.get_logger().info(
                f"  X0 = {self.start_x:.3f} m"
            )
            self.get_logger().info(
                f"  Y0 = {self.start_y:.3f} m"
            )
            self.get_logger().info(
                f"  Theta0 = {self.start_theta:.3f} rad"
            )

    # ================================================================
    # Control PID
    # ================================================================

    def control_loop(self):
        now = self.get_clock().now()

        if not self.have_odom:
            self.publish_zero()
            return

        # ------------------------------------------------------------
        # Seguridad: pérdida de odometría
        # ------------------------------------------------------------
        if self.last_odom_time is None:
            self.publish_zero()
            return

        odom_age = (
            now - self.last_odom_time
        ).nanoseconds / 1e9

        if odom_age > self.odom_timeout:
            self.get_logger().warning(
                "Odometría perdida. Deteniendo robot."
            )
            self.publish_zero()

            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_theta.reset()
            return

        # ------------------------------------------------------------
        # dt
        # ------------------------------------------------------------
        dt = (
            now - self.last_control_time
        ).nanoseconds / 1e9

        self.last_control_time = now

        if dt <= 0.0 or dt > 0.5:
            dt = 0.02

        # ------------------------------------------------------------
        # Si ya terminó
        # ------------------------------------------------------------
        if self.goal_reached:
            self.publish_zero()
            return

        if (
            self.start_x is None
            or self.start_y is None
            or self.start_theta is None
        ):
            self.publish_zero()
            return

        # ------------------------------------------------------------
        # Objetivo ABSOLUTO
        #
        # El usuario da un desplazamiento relativo:
        #
        #   target_x = +1.0
        #
        # significa avanzar 1 metro en X del mundo desde el
        # punto donde comenzó el nodo.
        # ------------------------------------------------------------
        goal_x = self.start_x + self.target_x
        goal_y = self.start_y + self.target_y

        goal_theta = self.normalize_angle(
            self.start_theta + self.target_theta
        )

        # ------------------------------------------------------------
        # Error en coordenadas globales
        # ------------------------------------------------------------
        error_x_global = goal_x - self.current_x
        error_y_global = goal_y - self.current_y

        error_theta = self.normalize_angle(
            goal_theta - self.current_theta
        )

        distance_error = math.hypot(
            error_x_global,
            error_y_global,
        )

        # ------------------------------------------------------------
        # Comprobar si llegó
        # ------------------------------------------------------------
        if (
            distance_error <= self.position_tolerance
            and abs(error_theta) <= self.theta_tolerance
        ):
            self.publish_zero()

            self.goal_reached = True

            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_theta.reset()

            self.get_logger().info(
                "========================================"
            )
            self.get_logger().info(
                "OBJETIVO ALCANZADO"
            )
            self.get_logger().info(
                f"Error X: {error_x_global:.4f} m"
            )
            self.get_logger().info(
                f"Error Y: {error_y_global:.4f} m"
            )
            self.get_logger().info(
                f"Error theta: {error_theta:.4f} rad"
            )
            self.get_logger().info(
                "========================================"
            )
            return

        # ------------------------------------------------------------
        # PID de posición
        #
        # Las salidas vx_global y vy_global representan la velocidad
        # deseada en coordenadas del mundo.
        # ------------------------------------------------------------
        vx_global = self.pid_x.update(
            error_x_global,
            dt,
        )

        vy_global = self.pid_y.update(
            error_y_global,
            dt,
        )

        w = self.pid_theta.update(
            error_theta,
            dt,
        )

        # ------------------------------------------------------------
        # Convertir velocidad GLOBAL -> BODY
        #
        # Para publicar geometry_msgs/Twist:
        #
        #   linear.x = velocidad hacia delante
        #   linear.y = velocidad lateral
        #
        # La transformación es:
        #
        # [vx_body]   [ cos(theta)  sin(theta)] [vx_global]
        # [vy_body] = [-sin(theta)  cos(theta)] [vy_global]
        # ------------------------------------------------------------
        c = math.cos(self.current_theta)
        s = math.sin(self.current_theta)

        vx_body = (
            c * vx_global
            + s * vy_global
        )

        vy_body = (
            -s * vx_global
            + c * vy_global
        )

        # ------------------------------------------------------------
        # Límites finales de seguridad
        # ------------------------------------------------------------
        vx_body = max(
            -self.max_vx,
            min(vx_body, self.max_vx),
        )

        vy_body = max(
            -self.max_vy,
            min(vy_body, self.max_vy),
        )

        w = max(
            -self.max_w,
            min(w, self.max_w),
        )

        # Velocidad mínima durante el movimiento.
        # Si la salida PID es cero, permanece en cero para poder detenerse.
        vx_body = self.apply_min_speed(vx_body, self.min_vx, self.max_vx)
        vy_body = self.apply_min_speed(vy_body, self.min_vy, self.max_vy)
        w = self.apply_min_speed(w, self.min_w, self.max_w)

        # ------------------------------------------------------------
        # Publicar /cmd_vel
        # ------------------------------------------------------------
        cmd = Twist()

        cmd.linear.x = vx_body
        cmd.linear.y = vy_body
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = w

        self.cmd_pub.publish(cmd)

        # ------------------------------------------------------------
        # Información periódica
        # ------------------------------------------------------------
        if not hasattr(self, "_log_counter"):
            self._log_counter = 0

        self._log_counter += 1

        if self._log_counter >= 50:
            self._log_counter = 0

            self.get_logger().info(
                f"Error: "
                f"X={error_x_global:.3f} m, "
                f"Y={error_y_global:.3f} m, "
                f"Theta={error_theta:.3f} rad | "
                f"Cmd: "
                f"vx={vx_body:.3f}, "
                f"vy={vy_body:.3f}, "
                f"w={w:.3f}"
            )

    # ================================================================
    # Shutdown
    # ================================================================

    def destroy_node(self):
        self.publish_zero()
        self.get_logger().info(
            "Robot detenido. Cerrando controlador."
        )
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = PositionController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

