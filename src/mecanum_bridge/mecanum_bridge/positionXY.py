#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math


class XYPositionController(Node):
    def __init__(self):
        super().__init__('xy_position_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Meta deseada en metros
        self.x_goal = 0.0
        self.y_goal = 1.0

        # Ganancias proporcionales
        self.kp_x = 0.45
        self.kp_y = 0.45

        # Velocidad máxima
        self.vmax_x = 0.15
        self.vmax_y = 0.15

        # Tolerancia de posición
        self.tolerance = 0.10

        self.get_logger().info('Controlador XY iniciado')

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        error_x_global = self.x_goal - x
        error_y_global = self.y_goal - y

        distance = math.sqrt(error_x_global**2 + error_y_global**2)

        cmd = Twist()

        if distance < self.tolerance:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info('Meta alcanzada')
            return

        # Convertir error global al marco del robot
        error_x_robot = math.cos(yaw) * error_x_global + math.sin(yaw) * error_y_global
        error_y_robot = -math.sin(yaw) * error_x_global + math.cos(yaw) * error_y_global

        vx = self.kp_x * error_x_robot
        vy = self.kp_y * error_y_robot

        vx = max(min(vx, self.vmax_x), -self.vmax_x)
        vy = max(min(vy, self.vmax_y), -self.vmax_y)

        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = XYPositionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
