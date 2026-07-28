#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
Control por teclado para robot mecanum

Movimiento:
  W / S   avanzar / retroceder
  A / D   desplazamiento lateral izquierda / derecha
  Q / E   giro izquierda / derecha
  X       detener

Velocidad:
  R       aumentar velocidad
  F       disminuir velocidad

  Ctrl+C  salir
"""


class TeleopMecanumKeyboard(Node):
    def __init__(self) -> None:
        super().__init__("teleop_mecanum_keyboard")

        self.declare_parameter("output_topic", "/cmd_vel_joy")
        self.declare_parameter("linear_speed", 0.15)
        self.declare_parameter("angular_speed", 0.50)
        self.declare_parameter("speed_step", 0.05)
        self.declare_parameter("min_linear_speed", 0.05)
        self.declare_parameter("max_linear_speed", 0.60)

        self.output_topic = str(
            self.get_parameter("output_topic").value
        )
        self.linear_speed = float(
            self.get_parameter("linear_speed").value
        )
        self.angular_speed = float(
            self.get_parameter("angular_speed").value
        )
        self.speed_step = float(
            self.get_parameter("speed_step").value
        )
        self.min_linear_speed = float(
            self.get_parameter("min_linear_speed").value
        )
        self.max_linear_speed = float(
            self.get_parameter("max_linear_speed").value
        )

        self.publisher = self.create_publisher(
            Twist,
            self.output_topic,
            10,
        )

        self.get_logger().info(
            f"Teleoperación por teclado iniciada: {self.output_topic}"
        )

    def publish_velocity(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        wz: float = 0.0,
    ) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.publisher.publish(msg)

    def stop(self) -> None:
        self.publish_velocity()

    def process_key(self, key: str) -> None:
        key = key.lower()

        if key == "w":
            self.publish_velocity(vx=self.linear_speed)

        elif key == "s":
            self.publish_velocity(vx=-self.linear_speed)

        elif key == "a":
            self.publish_velocity(vy=self.linear_speed)

        elif key == "d":
            self.publish_velocity(vy=-self.linear_speed)

        elif key == "q":
            self.publish_velocity(wz=self.angular_speed)

        elif key == "e":
            self.publish_velocity(wz=-self.angular_speed)

        elif key in ("x", " "):
            self.stop()

        elif key == "r":
            self.linear_speed = min(
                self.linear_speed + self.speed_step,
                self.max_linear_speed,
            )
            self.angular_speed = min(
                self.angular_speed + 0.10,
                2.0,
            )
            self.get_logger().info(
                f"Velocidad: {self.linear_speed:.2f} m/s, "
                f"giro: {self.angular_speed:.2f} rad/s"
            )

        elif key == "f":
            self.linear_speed = max(
                self.linear_speed - self.speed_step,
                self.min_linear_speed,
            )
            self.angular_speed = max(
                self.angular_speed - 0.10,
                0.10,
            )
            self.get_logger().info(
                f"Velocidad: {self.linear_speed:.2f} m/s, "
                f"giro: {self.angular_speed:.2f} rad/s"
            )


def read_key(settings) -> str:
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], 0.1)

    if ready:
        key = sys.stdin.read(1)
    else:
        key = ""

    termios.tcsetattr(
        sys.stdin,
        termios.TCSADRAIN,
        settings,
    )
    return key


def main(args=None) -> None:
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = TeleopMecanumKeyboard()

    print(HELP)
    print(
        f"Velocidad inicial: {node.linear_speed:.2f} m/s | "
        f"Giro: {node.angular_speed:.2f} rad/s"
    )

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key(settings)

            if key == "\x03":  # Ctrl+C
                break

            if key:
                node.process_key(key)

    except Exception as exc:
        node.get_logger().error(f"Error: {exc}")

    finally:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            settings,
        )

        # Publicar parada antes de cerrar ROS.
        if rclpy.ok():
            node.stop()
            rclpy.spin_once(node, timeout_sec=0.05)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
