"""Puente ROS 2 <-> PIC (serial 9600 8N1, comandos ASCII).

ROS -> PIC
  * /satelite/look_angles (geometry_msgs/Twist, de n2yo_connect):
        angular.z = azimut_deg, angular.y = elevacion_deg  ->  linea  G<az>,<el>
    (source:=twist permite elegir otro topico/campos, ver parametros)
  * /satelite/cmd (String): se envia tal cual al PIC (A90, E30, P0.8, S, Z, X...)
PIC -> ROS
  * Cada linea que manda el PIC se publica en /satelite/rx (String).
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
import serial

MAX_LINE = 23   # LARGO_LINEA (24) del firmware menos el '\0'
VALID_FIELDS = {f'{g}.{a}' for g in ('linear', 'angular') for a in 'xyz'}


def format_goto(az, el):
    """Comando G para el firmware. Siempre decimal fijo (sin notacion cientifica)."""
    return f'G{az:.2f},{el:.2f}'


def get_field(twist, path):
    obj = twist
    for part in path.split('.'):
        obj = getattr(obj, part)
    return float(obj)


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('source', 'look_angles')   # 'look_angles' | 'twist'
        self.declare_parameter('topic_in', '')            # '' = segun source
        self.declare_parameter('min_elevation_deg', -90.0)  # -90 = sin filtro
        self.declare_parameter('topic_cmd', '/satelite/cmd')
        self.declare_parameter('topic_rx', '/satelite/rx')
        self.declare_parameter('stamped', False)
        self.declare_parameter('az_field', 'angular.z')
        self.declare_parameter('el_field', 'angular.y')
        self.declare_parameter('units', 'deg')
        self.declare_parameter('min_period_s', 0.1)
        self.declare_parameter('deadband_deg', 0.05)
        self.declare_parameter('reconnect_s', 2.0)

        p = self.get_parameter
        self.port = p('port').value
        self.baud = p('baud').value
        self.az_field = p('az_field').value
        self.el_field = p('el_field').value
        self.units = p('units').value
        self.deadband = p('deadband_deg').value
        self.stamped = p('stamped').value
        self.source = p('source').value
        self.min_el = p('min_elevation_deg').value
        if self.source not in ('look_angles', 'twist'):
            raise ValueError('source debe ser "look_angles" o "twist"')
        topic_in = p('topic_in').value or (
            '/satelite/look_angles' if self.source == 'look_angles' else '/satelite')

        for f in (self.az_field, self.el_field):
            if f not in VALID_FIELDS:
                raise ValueError(f'Campo invalido "{f}". Usa uno de: {sorted(VALID_FIELDS)}')
        if self.units not in ('deg', 'rad'):
            raise ValueError('units debe ser "deg" o "rad"')

        self.ser = None
        self.buf = b''
        self.pending = None        # (az, el) mas reciente recibido por ROS
        self.last_sent = None      # (az, el) ultimo enviado al PIC

        if self.source == 'look_angles':
            self.create_subscription(Twist, topic_in, self.on_look_angles, 10)
        else:
            twist_type = TwistStamped if self.stamped else Twist
            self.create_subscription(twist_type, topic_in, self.on_twist, 10)
        self.create_subscription(String, p('topic_cmd').value, self.on_cmd, 10)
        self.pub_rx = self.create_publisher(String, p('topic_rx').value, 10)

        self.create_timer(0.01, self.read_serial)
        self.create_timer(p('min_period_s').value, self.flush_goto)
        self.create_timer(p('reconnect_s').value, self.ensure_open)
        self.ensure_open()

        self.get_logger().info(
            f'Puente listo: {self.port}@{self.baud} | '
            f'{topic_in} ({self.source}) [{self.units}] -> G | '
            f'{p("topic_cmd").value} -> serial | serial -> {p("topic_rx").value}')

    # ---------- Serial ----------
    def ensure_open(self):
        if self.ser is not None and self.ser.is_open:
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.buf = b''
            self.get_logger().info(f'Puerto {self.port} abierto')
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().warn(f'No se pudo abrir {self.port}: {e}',
                                   throttle_duration_sec=10.0)

    def _close(self):
        try:
            if self.ser:
                self.ser.close()
        finally:
            self.ser = None

    def send_line(self, text):
        """Envia una linea completa en UNA sola escritura (el firmware cierra la
        linea si pasan 50 ms sin recibir nada)."""
        text = text.strip()
        if not text:
            return False
        if len(text) > MAX_LINE:
            self.get_logger().warn(f'Comando demasiado largo ({len(text)}>{MAX_LINE}): "{text}"')
            return False
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial cerrado, comando descartado',
                                   throttle_duration_sec=5.0)
            return False
        try:
            self.ser.write((text + '\n').encode('ascii', errors='ignore'))
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Error escribiendo serial: {e}')
            self._close()
            return False
        self.get_logger().info(f'-> PIC: {text}')
        return True

    def read_serial(self):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Error leyendo serial: {e}')
            self._close()
            return
        if not data:
            return
        self.buf += data
        while b'\n' in self.buf:
            raw, self.buf = self.buf.split(b'\n', 1)
            text = raw.decode('ascii', errors='ignore').strip()
            if text:
                self.pub_rx.publish(String(data=text))
        if len(self.buf) > 4096:
            self.buf = b''

    # ---------- ROS -> PIC ----------
    def on_twist(self, msg):
        twist = msg.twist if self.stamped else msg
        self.set_target(get_field(twist, self.az_field),
                        get_field(twist, self.el_field))

    def on_look_angles(self, msg):
        # Contrato n2yo_connect: angular.z = azimut, angular.y = elevacion (grados)
        self.set_target(float(msg.angular.z), float(msg.angular.y))

    def set_target(self, az, el):
        if not (math.isfinite(az) and math.isfinite(el)):
            return
        if self.units == 'rad':
            az, el = math.degrees(az), math.degrees(el)
        if el < self.min_el:       # p. ej. satelite bajo el horizonte
            return
        self.pending = (az, el)

    def flush_goto(self):
        """Limita la tasa y evita reenviar valores casi iguales."""
        if self.pending is None:
            return
        az, el = self.pending
        self.pending = None
        if self.last_sent is not None:
            if (abs(az - self.last_sent[0]) < self.deadband and
                    abs(el - self.last_sent[1]) < self.deadband):
                return
        if self.send_line(format_goto(az, el)):
            self.last_sent = (az, el)

    def on_cmd(self, msg):
        self.send_line(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()