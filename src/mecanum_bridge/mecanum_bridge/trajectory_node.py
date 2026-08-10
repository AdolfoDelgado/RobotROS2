#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class TrajectoryNode(Node):

    def __init__(self):
        super().__init__('trajectory_node')

        # Suscripción a la odometría filtrada por EKF
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        # Publicador de trayectoria
        self.path_pub = self.create_publisher(
            Path,
            '/robot_path',
            10
        )

        # Mensaje Path
        self.path = Path()
        self.path.header.frame_id = 'odom'

        # Para evitar guardar demasiados puntos
        self.last_x = None
        self.last_y = None

        # Distancia mínima entre puntos consecutivos
        self.min_distance = 0.02  # 2 cm

        self.get_logger().info(
            'Trajectory Node iniciado. '
            'Escuchando /odometry/filtered'
        )

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Primera posición
        if self.last_x is None:
            self.last_x = x
            self.last_y = y

            self.add_pose(msg)
            return

        # Distancia desde el último punto almacenado
        dx = x - self.last_x
        dy = y - self.last_y

        distance = (dx ** 2 + dy ** 2) ** 0.5

        # Solo agregamos un punto si el robot se ha movido
        # una distancia mínima
        if distance >= self.min_distance:

            self.add_pose(msg)

            self.last_x = x
            self.last_y = y

    def add_pose(self, msg):

        pose = PoseStamped()

        # Timestamp
        pose.header.stamp = msg.header.stamp

        # Frame de referencia
        pose.header.frame_id = 'odom'

        # Copiar posición y orientación
        pose.pose = msg.pose.pose

        # Agregar al Path
        self.path.poses.append(pose)

        # Actualizar header del Path
        self.path.header.stamp = msg.header.stamp

        # Publicar trayectoria
        self.path_pub.publish(self.path)


def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
