#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyMecanum(Node):

    def __init__(self):
        super().__init__('joy_mecanum')
        # PARAMETROS
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_joy')
        # Ejes
        self.declare_parameter('axis_vx', 1)
        self.declare_parameter('axis_vy', 0)
        self.declare_parameter('axis_wz', 3)
        # Escalas
        self.declare_parameter('scale_vx', 0.35)
        self.declare_parameter('scale_vy', 0.35)
        self.declare_parameter('scale_wz', 0.8)
        # Deadzone
        self.declare_parameter('deadzone', 0.08)
        # Boton A
        self.declare_parameter('enable_button', 0)
        # Gatillos
        self.declare_parameter('rt_axis', 5)
        self.declare_parameter('lt_axis', 4)
        # Incremento de velocidad
        self.declare_parameter('speed_step', 0.05)
        # OBTENER PARAMETROS
        joy_topic = self.get_parameter('joy_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.axis_vx = self.get_parameter('axis_vx').value
        self.axis_vy = self.get_parameter('axis_vy').value
        self.axis_wz = self.get_parameter('axis_wz').value

        self.scale_vx = self.get_parameter('scale_vx').value
        self.scale_vy = self.get_parameter('scale_vy').value
        self.scale_wz = self.get_parameter('scale_wz').value

        self.deadzone = self.get_parameter('deadzone').value

        self.enable_button = self.get_parameter('enable_button').value

        self.rt_axis = self.get_parameter('rt_axis').value
        self.lt_axis = self.get_parameter('lt_axis').value

        self.speed_step = self.get_parameter('speed_step').value

        
        # ESTADO
        self.enabled = False

        # Multiplicador de velocidad
        self.speed_multiplier = 1.0

        # Último comando enviado
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_wz = 0.0

        # Estado anterior del botón A
        self.last_enable_state = False

        # Últimos valores de los gatillos
        self.last_rt = None
        self.last_lt = None

        # ROS
        self.cmd_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )

        self.joy_sub = self.create_subscription(
            Joy,
            joy_topic,
            self.joy_callback,
            10
        )

        self.get_logger().info(
            f'Joystick iniciado: {joy_topic} -> {cmd_vel_topic}'
        )

   
    # DEADZONE
    def apply_deadzone(self, value):

        if abs(value) < self.deadzone:
            return 0.0

        return value

  
    # PUBLICAR VELOCIDAD
    def publish_velocity(self, vx, vy, wz):

        # Si exactamente no cambió, no enviamos nada
        if (
            vx == self.last_vx and
            vy == self.last_vy and
            wz == self.last_wz
        ):
            return

        msg = Twist()

        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz

        self.cmd_pub.publish(msg)

        self.last_vx = vx
        self.last_vy = vy
        self.last_wz = wz

  
    # STOP
    def publish_stop(self):

        # Si ya estamos en cero no hace falta volver a enviarlo
        if (
            self.last_vx == 0.0 and
            self.last_vy == 0.0 and
            self.last_wz == 0.0
        ):
            return

        msg = Twist()

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0

        self.cmd_pub.publish(msg)

        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_wz = 0.0

        self.get_logger().info('STOP')

   
    # CALLBACK JOYSTICK
    def joy_callback(self, msg):

        # ----------------------------------------------------------
        # BOTON A
        # ----------------------------------------------------------
        if self.enable_button >= len(msg.buttons):
            self.get_logger().error(
                f'El botón {self.enable_button} no existe'
            )
            return

        a_pressed = msg.buttons[self.enable_button] == 1
        # ----------------------------------------------------------
        # A FUE PRESIONADA
        # ----------------------------------------------------------
        if a_pressed and not self.last_enable_state:

            self.enabled = True

            self.get_logger().info('CONTROL HABILITADO')
        # ----------------------------------------------------------
        # A FUE SOLTADA
        # ----------------------------------------------------------
        if not a_pressed and self.last_enable_state:

            self.enabled = False

            # STOP UNA SOLA VEZ
            self.publish_stop()

            self.get_logger().info('CONTROL DESHABILITADO')

        self.last_enable_state = a_pressed
        # ----------------------------------------------------------
        # SI A NO ESTA PRESIONADA
        # ----------------------------------------------------------
        if not self.enabled:
            return
        # EJES
        if self.axis_vx >= len(msg.axes):
            return

        if self.axis_vy >= len(msg.axes):
            return

        if self.axis_wz >= len(msg.axes):
            return

        vx_axis = self.apply_deadzone(
            msg.axes[self.axis_vx]
        )

        vy_axis = self.apply_deadzone(
            msg.axes[self.axis_vy]
        )

        wz_axis = self.apply_deadzone(
            msg.axes[self.axis_wz]
        )

        
        # GATILLOS
        rt = None
        lt = None

        if self.rt_axis < len(msg.axes):
            rt = msg.axes[self.rt_axis]

        if self.lt_axis < len(msg.axes):
            lt = msg.axes[self.lt_axis]
        # ----------------------------------------------------------
        # RT
        # ----------------------------------------------------------
        if rt is not None:

            if self.last_rt is None:
                self.last_rt = rt

            elif abs(rt - self.last_rt) > 0.01:

                # RT aumenta velocidad
                if rt > self.last_rt:

                    self.speed_multiplier += self.speed_step

                self.last_rt = rt
        # ----------------------------------------------------------
        # LT
        # ----------------------------------------------------------
        if lt is not None:

            if self.last_lt is None:
                self.last_lt = lt

            elif abs(lt - self.last_lt) > 0.01:

                # LT disminuye velocidad
                if lt > self.last_lt:

                    self.speed_multiplier -= self.speed_step

                self.last_lt = lt
        # Limitar velocidad
        self.speed_multiplier = max(
            0.1,
            min(self.speed_multiplier, 2.0)
        )

        # VELOCIDADES
        vx = (
            vx_axis *
            self.scale_vx *
            self.speed_multiplier
        )

        vy = (
            vy_axis *
            self.scale_vy *
            self.speed_multiplier
        )

        wz = (
            wz_axis *
            self.scale_wz *
            self.speed_multiplier
        )

        
        # PUBLICAR SOLO SI CAMBIO
        self.publish_velocity(
            vx,
            vy,
            wz
        )

def main(args=None):

    rclpy.init(args=args)

    node = JoyMecanum()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        # Intentar detener el robot al cerrar el nodo
        node.publish_stop()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
