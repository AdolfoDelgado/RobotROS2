#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyMecanum(Node):
    """
    Convierte /joy en velocidades para un robot mecanum.

    Mapeo predeterminado tipo Xbox:
      stick izquierdo vertical   -> vx
      stick izquierdo horizontal -> vy
      stick derecho horizontal   -> wz
      RT                          -> aumenta el nivel de velocidad
      LT                          -> disminuye el nivel de velocidad

    RT y LT cambian el nivel una sola vez por cada pulsación. Esto evita que
    el nivel cambie muchas veces mientras el gatillo permanece presionado.
    """

    def __init__(self) -> None:
        super().__init__("joy_mecanum")

        # Ejes usuales del driver joy para un mando Xbox.
        self.declare_parameter("axis_vx", 1)
        self.declare_parameter("axis_vy", 0)
        self.declare_parameter("axis_wz", 3)
        self.declare_parameter("axis_lt", 2)
        self.declare_parameter("axis_rt", 5)

        # Algunos mandos reportan los gatillos sueltos en +1 y presionados en -1.
        self.declare_parameter("trigger_released_value", 1.0)
        self.declare_parameter("trigger_pressed_value", -1.0)
        self.declare_parameter("trigger_threshold", 0.65)

        self.declare_parameter("max_vx", 0.40)
        self.declare_parameter("max_vy", 0.40)
        self.declare_parameter("max_wz", 1.00)

        self.declare_parameter("speed_levels", [0.20, 0.35, 0.50, 0.70, 1.00])
        self.declare_parameter("initial_speed_level", 1)

        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("joy_timeout", 0.5)
        self.declare_parameter("output_topic", "/cmd_vel_joy")

        self.axis_vx = int(self.get_parameter("axis_vx").value)
        self.axis_vy = int(self.get_parameter("axis_vy").value)
        self.axis_wz = int(self.get_parameter("axis_wz").value)
        self.axis_lt = int(self.get_parameter("axis_lt").value)
        self.axis_rt = int(self.get_parameter("axis_rt").value)

        self.trigger_released = float(
            self.get_parameter("trigger_released_value").value
        )
        self.trigger_pressed = float(
            self.get_parameter("trigger_pressed_value").value
        )
        self.trigger_threshold = float(
            self.get_parameter("trigger_threshold").value
        )

        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_wz = float(self.get_parameter("max_wz").value)

        self.speed_levels = [
            float(value)
            for value in self.get_parameter("speed_levels").value
        ]

        initial_level = int(
            self.get_parameter("initial_speed_level").value
        )
        self.speed_level = max(
            0,
            min(initial_level, len(self.speed_levels) - 1),
        )

        self.deadzone = float(self.get_parameter("deadzone").value)
        self.joy_timeout = float(
            self.get_parameter("joy_timeout").value
        )
        publish_rate = float(
            self.get_parameter("publish_rate").value
        )
        output_topic = str(
            self.get_parameter("output_topic").value
        )

        self.latest_joy = None
        self.last_joy_time = None

        # Bloqueos para que cada gatillo produzca un solo cambio por pulsación.
        self.rt_latched = False
        self.lt_latched = False

        self.joy_sub = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            output_topic,
            10,
        )

        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_cmd_vel,
        )

        self.get_logger().info(
            f"Joystick iniciado: /joy -> {output_topic}"
        )
        self.log_speed_level()

    @staticmethod
    def valid_index(values, index: int) -> bool:
        return 0 <= index < len(values)

    def apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def trigger_amount(self, raw_value: float) -> float:
        """
        Convierte el eje del gatillo a 0.0..1.0.

        Funciona tanto si el mando usa:
          suelto=+1, presionado=-1
        como si se cambian esos valores mediante parámetros.
        """
        span = self.trigger_pressed - self.trigger_released
        if abs(span) < 1.0e-9:
            return 0.0

        amount = (raw_value - self.trigger_released) / span
        return max(0.0, min(1.0, amount))

    def joy_callback(self, msg: Joy) -> None:
        self.latest_joy = msg
        self.last_joy_time = self.get_clock().now()

        if not self.valid_index(msg.axes, self.axis_rt):
            return
        if not self.valid_index(msg.axes, self.axis_lt):
            return

        rt_pressed = (
            self.trigger_amount(msg.axes[self.axis_rt])
            >= self.trigger_threshold
        )
        lt_pressed = (
            self.trigger_amount(msg.axes[self.axis_lt])
            >= self.trigger_threshold
        )

        # Si ambos están presionados, no cambia el nivel.
        if rt_pressed and lt_pressed:
            self.rt_latched = True
            self.lt_latched = True
            return

        if rt_pressed and not self.rt_latched:
            self.speed_level = min(
                self.speed_level + 1,
                len(self.speed_levels) - 1,
            )
            self.log_speed_level()

        if lt_pressed and not self.lt_latched:
            self.speed_level = max(self.speed_level - 1, 0)
            self.log_speed_level()

        self.rt_latched = rt_pressed
        self.lt_latched = lt_pressed

    def log_speed_level(self) -> None:
        factor = self.speed_levels[self.speed_level]
        self.get_logger().info(
            f"Nivel {self.speed_level + 1}/{len(self.speed_levels)}: "
            f"{factor * 100:.0f}%"
        )

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def publish_cmd_vel(self) -> None:
        if self.latest_joy is None or self.last_joy_time is None:
            self.publish_stop()
            return

        elapsed = (
            self.get_clock().now() - self.last_joy_time
        ).nanoseconds / 1.0e9

        # Seguridad: si se desconecta el mando, manda cero.
        if elapsed > self.joy_timeout:
            self.publish_stop()
            return

        joy = self.latest_joy
        required_axes = [self.axis_vx, self.axis_vy, self.axis_wz]

        if not all(
            self.valid_index(joy.axes, index)
            for index in required_axes
        ):
            self.get_logger().warning(
                "Los índices de movimiento no existen en /joy",
                throttle_duration_sec=2.0,
            )
            self.publish_stop()
            return

        factor = self.speed_levels[self.speed_level]

        vx_input = self.apply_deadzone(joy.axes[self.axis_vx])
        vy_input = self.apply_deadzone(joy.axes[self.axis_vy])
        wz_input = self.apply_deadzone(joy.axes[self.axis_wz])

        cmd = Twist()
        cmd.linear.x = self.max_vx * factor * vx_input
        cmd.linear.y = self.max_vy * factor * vy_input
        cmd.angular.z = self.max_wz * factor * wz_input

        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyMecanum()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
