import rclpy
import os
import requests
import csv
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dataclasses import dataclass

#=== Global Variables ===
@dataclass
class Satelite_data:
    latitud_satelite: float = 0.0
    longitud_satelite: float = 0.0
    azimuth: float = 0.0
    elevacion: float = 0.0
    timestamp: float = 0.0
    eclipsed: bool = 1


satelite_lista = []
node = None
pub = None
pos = 0
aux = True


#=== Variables Config ===
API_KEY = os.environ.get('N2YO_API_KEY', '')
NORAD_ID = 25544 #ISS ID
N_SEGUNDOS = 300

LATITUD = 4.627125
LONGITUD = -74.064183
ALTITUD =2600           #Coordenadas de edificio ingenieria 

CARPETA = os.path.expanduser("~/Projects/Interestelar/src/interestelar_pkg/pos_satelite")
FILE = os.path.join(CARPETA, "historial_satelite.csv")
INTERVALO = 290
CAMPOS = ['satlatitude', 'satlongitude', 'sataltitude',
          'azimuth', 'elevation', 'timestamp', 'eclipsed']




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

#=================== loac csv file ==========================
# Read teh cvs file and return a diccionaries list 
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
    satelite_data.azimuth = data["azimuth"]
    satelite_data.elevacion = data["elevation"]
    satelite_data.timestamp = data["timestamp"]
    satelite_data.eclipsed = data["eclipsed"]

    return satelite_data

#=================== Take Positions ==========================
def config_positions(lista):
    global satelite_lista
   
    
    for i in lista:
        satelite_lista.append( read_position(i))


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
#============================ class to Twist ============================== 
def to_Twist(sat):
    data = Twist()
    
    data.linear.x = sat.latitud_satelite
    data.linear.y = sat.longitud_satelite
    data.linear.z = float(sat.eclipsed)
    data.angular.x = sat.azimuth
    data.angular.y = sat.elevacion
    data.angular.z = float(sat.timestamp)

    return data

#============================ Timmer 1 read data ==============================
def timmer_read_data():
    global pos
    # 1.Write data  - 2.Read saved data  - 3.Read live data
    mode_comunication = 3
    pos = 0

    #Interface
    interface(mode_comunication)

#============================ Timmer 2 publish ==============================
def callTime():
    global pub
    global pos

    if pos >= len(satelite_lista):
        return
    
    pub.publish(to_Twist(satelite_lista[pos]))
    pos = pos + 1
    

#============================   main     ==============================
def main():
    global node
    global pub

    #Init ROS2
    rclpy.init()

    #create a node
    node = rclpy.create_node("n2yo_api")
    pub = node.create_publisher(Twist,"/satelite",10)

    print("==================================================")
    print("   I N T E R E S T E L A R     P R O Y E C E T")
    print("==================================================")

    #Create a timer
    node.create_timer(10,timmer_read_data)
    node.create_timer(1,callTime)
    

    #create a spincallTime
    rclpy.spin(node)

    #shutdown
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()