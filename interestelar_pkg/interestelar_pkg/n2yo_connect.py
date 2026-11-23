import rclpy
import os
import requests
import csv
import time
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64MultiArray
from dataclasses import dataclass

#=== Global Variables ===
@dataclass
class Satelite_data:
    latitud_satelite: float = 0.0
    longitud_satelite: float = 0.0
    altitud_satelite: float = 0.0   # km (como lo entrega N2YO)
    azimuth: float = 0.0
    elevacion: float = 0.0
    timestamp: float = 0.0
    eclipsed: bool = True


satelite_lista = []
node = None
pub_fix = None      # /satelite              (sensor_msgs/NavSatFix)
pub_look = None     # /satelite/look_angles  (std_msgs/Float64MultiArray)
pos = 0
aux = True


#=== Variables Config ===
API_KEY = os.environ.get('N2YO_API_KEY', '')
NORAD_ID = 25544 #ISS ID
N_SEGUNDOS = 300

LATITUD = 4.627125
LONGITUD = -74.064183
ALTITUD = 2600           #Coordenadas de edificio ingenieria

CARPETA = os.path.expanduser("~/Projects/Interestelar/src/interestelar_pkg/pos_satelite")
FILE = os.path.join(CARPETA, "historial_satelite.csv")
INTERVALO = 290          # segundos entre consultas a la API
CAMPOS = ['satlatitude', 'satlongitude', 'sataltitude',
          'azimuth', 'elevation', 'timestamp', 'eclipsed']

# Modo de comunicacion: 1.Write data  - 2.Read saved data  - 3.Read live data
MODE_COMUNICATION = 3


#====================== API Connection =====================
def api_connect():
    global aux
    url = (f'https://api.n2yo.com/rest/v1/satellite/positions/'
           f'{NORAD_ID}/{LATITUD}/{LONGITUD}/{ALTITUD}/{N_SEGUNDOS}/&apiKey={API_KEY}')
    posi = requests.get(url, timeout=5.0)
    posi.raise_for_status()

    if aux: print("Conectado a la API con exito")

    return posi.json()


#====================== Read Saved Timestaps=====================
def read_saved_timestamps():

    guardados = set()
    #If exist file return the file, if not create a new
    if not os.path.exists(FILE):
        return guardados

    with open(FILE, 'r', newline='') as f:
        lector = csv.DictReader(f)
        for fila in lector:
            guardados.add(int(fila['timestamp']))

    return guardados


#=================== Save Positions ==========================
def save_pos(lista):

    guardados = read_saved_timestamps()
    archivo_nuevo = not os.path.exists(FILE)

    nuevas = []
    for p in lista:
        if p['timestamp'] not in guardados:
            nuevas.append(p)

    os.makedirs(CARPETA, exist_ok=True)
    with open(FILE, 'a', newline='') as f:
        escritor = csv.DictWriter(f, fieldnames=CAMPOS, extrasaction='ignore')
        if archivo_nuevo:
            escritor.writeheader()
        escritor.writerows(nuevas)

    return len(nuevas)

#=================== load csv file ==========================
# Read the csv file and return a dictionaries list
def load_csv():
    lista = []
    if not os.path.exists(FILE):
        return lista

    with open(FILE, 'r', newline='') as f:
        for fila in csv.DictReader(f):
            p = {}
            p['timestamp'] = float(fila['timestamp'])
            p['satlatitude'] = float(fila['satlatitude'])
            p['satlongitude'] = float(fila['satlongitude'])
            p['sataltitude'] = float(fila['sataltitude'])
            p['azimuth'] = float(fila['azimuth'])
            p['elevation'] = float(fila['elevation'])
            p['eclipsed'] = (fila['eclipsed'] == 'True')
            lista.append(p)

    lista.sort(key=lambda x: x['timestamp'])
    return lista

#=================== Take Positions ==========================
def read_position(data):
    satelite_data = Satelite_data()

    satelite_data.latitud_satelite = data["satlatitude"]
    satelite_data.longitud_satelite = data["satlongitude"]
    satelite_data.altitud_satelite = data["sataltitude"]
    satelite_data.azimuth = data["azimuth"]
    satelite_data.elevacion = data["elevation"]
    satelite_data.timestamp = data["timestamp"]
    satelite_data.eclipsed = data["eclipsed"]

    return satelite_data

#=================== Config Positions ==========================
def config_positions(lista):
    global satelite_lista
    # Se reemplaza la lista completa para no acumular posiciones repetidas
    satelite_lista = [read_position(i) for i in lista]


#============================ Output data ==============================
def show_data(datos):
    primero = datos[0]['timestamp']
    ultimo = datos[-1]['timestamp']

    fmt = '%Y-%m-%d %H:%M:%S'
    print(f'Posiciones leídas: {len(datos)}')
    print(f'Desde: {time.strftime(fmt, time.gmtime(primero))} UTC')
    print(f'Hasta: {time.strftime(fmt, time.gmtime(ultimo))} UTC')

#============================ Interface ==============================
def interface(input_command):
    global aux
    try:

        match input_command:
            #-----------------------------------------------------------
            case 1: #Write positions
                data = api_connect()
                aux = False

                if 'positions' not in data:
                    node.get_logger().error(f'Error de la API: {data}')
                    return
                else:
                    n = save_pos(data['positions'])

                    usadas = data['info']['transactionscount']
                    print(f'{time.strftime("%H:%M:%S")} | nuevas: {n} | '
                        f'consultas en la última hora: {usadas}')
            #-----------------------------------------------------------
            case 2: #Read positions
                datos = load_csv()
                if not datos:
                    print('No hay datos guardados todavía.')
                    return

                config_positions(datos)

            #-----------------------------------------------------------
            case 3:#Live positions
                data = api_connect()
                aux = False
                if 'positions' not in data:
                    print(f'Error de la API: {data}')
                else:
                    config_positions(data['positions'])
            #-----------------------------------------------------------
    except Exception as e:
        print(f'Error ({type(e).__name__}): {e}')
        return

#============================ class to NavSatFix ==============================
def to_NavSatFix(sat):
    msg = NavSatFix()

    # Timestamp del satelite (epoch UNIX) -> header.stamp
    msg.header.stamp.sec = int(sat.timestamp)
    msg.header.stamp.nanosec = int((sat.timestamp % 1) * 1e9)
    msg.header.frame_id = "satelite"

    msg.status.status = NavSatStatus.STATUS_FIX
    msg.status.service = NavSatStatus.SERVICE_GPS

    msg.latitude = float(sat.latitud_satelite)
    msg.longitude = float(sat.longitud_satelite)
    msg.altitude = float(sat.altitud_satelite) * 1000.0   # N2YO da km -> NavSatFix usa metros

    # No tenemos covarianza real
    msg.position_covariance = [0.0] * 9
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

    return msg

#============================ class to look angles ==============================
# data = [azimuth (deg), elevacion (deg), eclipsed (0.0 / 1.0)]
def to_look_angles(sat):
    msg = Float64MultiArray()
    msg.data = [
        float(sat.azimuth),
        float(sat.elevacion),
        float(sat.eclipsed),
    ]
    return msg

#============================ Timmer 1 read data ==============================
def timmer_read_data():
    global pos
    pos = 0

    #Interface
    interface(MODE_COMUNICATION)

#============================ Timmer 2 publish ==============================
def callTime():
    global pos

    if pos >= len(satelite_lista):
        return

    sat = satelite_lista[pos]

    pub_fix.publish(to_NavSatFix(sat))
    pub_look.publish(to_look_angles(sat))

    pos = pos + 1


#============================   main     ==============================
def main():
    global node
    global pub_fix
    global pub_look

    #Init ROS2
    rclpy.init()

    #create a node
    node = rclpy.create_node("n2yo_api")
    pub_fix = node.create_publisher(NavSatFix, "/satelite", 10)
    pub_look = node.create_publisher(Float64MultiArray, "/satelite/look_angles", 10)

    print("==================================================")
    print("   I N T E R E S T E L A R     P R O Y E C T")
    print("==================================================")

    # Primera lectura inmediata (sin esperar al primer timer)
    timmer_read_data()

    #Create timers
    node.create_timer(INTERVALO, timmer_read_data)   # consulta a la API
    node.create_timer(1, callTime)                   # publica 1 posicion por segundo

    #spin
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    #shutdown
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()