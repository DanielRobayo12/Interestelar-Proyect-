"""Publica /joint_states a partir de /satelite/look_angles para verlo en RViz.

Entrada
  * /satelite/look_angles (geometry_msgs/Twist, de n2yo_connect):
        angular.z = azimut (deg)   angular.y = elevacion (deg)
Salida
  * /joint_states (sensor_msgs/JointState) con las articulaciones del URDF:
        azimuth   (revolute, eje Z, 0..2*pi)
        elevacion (revolute, eje X, 0..2*pi)
    robot_state_publisher usa este topico para calcular el TF y RViz mueve el modelo.

Parametros utiles (ros2 run interestelar_pkg joints_pub --ros-args -p az_sign:=-1.0)
  * az_sign / el_sign        : +1.0 o -1.0 para invertir el sentido de giro
  * az_offset_deg / el_offset_deg : cero mecanico del modelo respecto al cero de la API
  * publish_rate_hz          : tasa a la que se republica el ultimo valor
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

TWO_PI = 2.0 * math.pi


class JointsPub(Node):
    def __init__(self):
        super().__init__('joints_pub')

        self.declare_parameter('topic_in', '/satelite/look_angles')
        self.declare_parameter('topic_out', '/joint_states')
        self.declare_parameter('az_joint', 'azimuth')
        self.declare_parameter('el_joint', 'elevacion')
        self.declare_parameter('az_sign', 1.0)
        self.declare_parameter('el_sign', 1.0)
        self.declare_parameter('az_offset_deg', 0.0)
        self.declare_parameter('el_offset_deg', 0.0)
        self.declare_parameter('publish_rate_hz', 20.0)

        p = self.get_parameter
        self.az_joint = p('az_joint').value
        self.el_joint = p('el_joint').value
        self.az_sign = float(p('az_sign').value)
        self.el_sign = float(p('el_sign').value)
        self.az_off = float(p('az_offset_deg').value)
        self.el_off = float(p('el_offset_deg').value)

        # Posicion inicial (rad) hasta recibir el primer mensaje
        self.az_rad = 0.0
        self.el_rad = 0.0

        self.pub = self.create_publisher(JointState, p('topic_out').value, 10)
        self.create_subscription(Twist, p('topic_in').value, self.on_look_angles, 10)
        self.create_timer(1.0 / float(p('publish_rate_hz').value), self.publish_joints)

        self.get_logger().info(
            f'{p("topic_in").value} (Twist) -> {p("topic_out").value} '
            f'[{self.az_joint}, {self.el_joint}]')

    def on_look_angles(self, msg):
        az = float(msg.angular.z)
        el = float(msg.angular.y)
        if not (math.isfinite(az) and math.isfinite(el)):
            return
        # Se normaliza a [0, 2*pi): los limites del URDF son 0..2*pi y una
        # elevacion negativa (bajo el horizonte) equivale a 360 + el.
        self.az_rad = math.radians(self.az_sign * az + self.az_off) % TWO_PI
        self.el_rad = math.radians(self.el_sign * el + self.el_off) % TWO_PI

    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.az_joint, self.el_joint]
        msg.position = [self.az_rad, self.el_rad]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointsPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
