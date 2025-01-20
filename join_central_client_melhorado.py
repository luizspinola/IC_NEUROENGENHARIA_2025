import os
import sys
import typing #typing não utilizado
import numpy as np
import pyqtgraph as pg
import numpy as np
from PyQt6 import QtCore, QtGui, QtWidgets #QtGui não utilizado
from PyQt6 import uic
from PyQt6.QtCore import QProcess, QObject, QThread, pyqtSlot as Slot, pyqtSignal, QMutex #QProcess não utilizado

import pyqtgraph.opengl as gl
import copy
import time #Não acessado
import struct #Não acessado
import socket
import commands
from data_handler import DataHandler #Não acessado
import util
import imu #Não acessado
from scipy.spatial.transform import Rotation as Rot
import math

from ahrs.filters import EKF
from ahrs.filters import FAMC
from ahrs.common.orientation import acc2q

DEFAULT_NETWORK_PREFIX = '172.30.1' # Endereço IPv4 adaptador ethernet
SVR_DATA_MC_IP = '230.0.0.0'
SVR_CFG_MC_IP = '230.0.0.1'
SEN_CFG_MC_IP = '230.0.0.2'
PORT = 50000

def get_default_ip():
    itf = '0.0.0.0' 
    itfs = socket.gethostbyname_ex(socket.gethostname())[2] # Gethostname retorna o nome do dispositivo a partir do qual 
                                                            # o Python está rodando. Gethostbyname_ex retorna uma tupla
                                                            # contendo o nome do host, outros names para o host e o IP
    for ip in itfs:
        if ip.startswith(DEFAULT_NETWORK_PREFIX):
            itf = ip
            break
    return itf

def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath(".")

    return os.path.join(base_path, relative_path)

            
class SaveData(QObject):
    def __init__(self):
        super().__init__()
        self.file = open("dados.imu", "wb")

    @Slot(int, np.ndarray)
    def update(self, sensor, data):
        self.file.write(sensor.to_bytes(4, 'big') + np.ndarray.tobytes(data))
    
    #@Slot()
    #def write_headline(self, sensor_list, freq):    pass

    def close_file(self):
        self.file.close()

class FormatData(QObject):
    trans = pyqtSignal(np.ndarray)
    trans_save = pyqtSignal(int, np.ndarray)
    quaternium = pyqtSignal(np.ndarray)
    # Valor válido para todas as instâncias dessa classe
    AFS_SEL = 0
    FS_SEL = 0
    FREQ = 500
    GRAV = 9.80665

    def __init__(self):
        super().__init__()
        self.sensor_limits()
        self.define_size_buffer()
        self.axis_id = 2 # Representando o 'AGM'
        self.qt_var_sensor = 9 # Representando as 9 informações a serem coletadas 
                               # (ax, ay, az, gx, gy, gz, mx, my, mz)
        self.qt_info_max = 11
        self.define_storage_data()
        self.sensor_show = 2

        self.define_var_kalman()

        self.roll = 0
        self.pitch = 0

        #Definição de variáveis para o filtro de Kalman extendido
        self.ekf = EKF(frequency = 500.0, magnetic_ref = 32)
        self.Q = []
        self.counter = 0

    def define_size_buffer(self):
        if FormatData.FREQ == 1:
            self.qt_data_queue = 1
        else:
            self.qt_data_queue = (30 * FormatData.FREQ)//500

    def define_storage_data (self):
        self.data_sensors = {}
        self.data_save_sensors = {}
        self.grp_sensors = {}
        for i in range(1,10):
            self.data_sensors[i] = np.full((self.qt_var_sensor, self.qt_data_queue), 40000.0)
            self.data_save_sensors[i] = np.full((self.qt_info_max, self.qt_data_queue), 40000)
            self.grp_sensors[i] = 0

    def define_var_kalman(self):
         self.X = np.array([1.0, 0.0, 0.0, 0.0])
         self.P = np.eye(4)
         self.R = np.eye(4) * 10.0
         self.Q = np.eye(4) * 1e-4
         self.C = np.eye(4)

    @Slot(int)
    def get_axis_id(self, axis_id):
        self.axis_id = axis_id
        self.qt_var_sensor = (self.axis_id + 1) * 3
        self.define_storage_data()

    def sensor_limits(self):
        accel_bits = 16
        gyr_bits = 16
        mag_bits = 16 # Está também em 16 (firmware e não hardware)

        self.accel_blimit = [-(2**(accel_bits-1)), (2**(accel_bits-1))-1]
        self.gyr_blimit = [-(2**(gyr_bits-1)), (2**(gyr_bits-1))-1]
        self.mag_blimit = [-(2**(mag_bits-1)), (2**(mag_bits-1))-1]

        match FormatData.AFS_SEL:
            case 0:
                self.accel_elimit = 2 # Unidade g
            case 1:
                self.accel_elimit = 4
            case 2:
                self.accel_elimit = 8
            case 3:
                self.accel_elimit = 16
            
        print(FormatData.GRAV)
        self.accel_elimit *= FormatData.GRAV
        
        match FormatData.FS_SEL:
            case 0:
                self.gyr_elimit = 250 # Unidade º/s
            case 1:
                self.gyr_elimit = 500
            case 2:
                self.gyr_elimit = 1000
            case 3:
                self.gyr_elimit = 2000
        
        self.mag_elimit = 4800 # Unidade uT
    
    @Slot(dict)
    def join_data(self, data):
        sensor = data.sensor_id
        position = data.index % self.qt_data_queue
        qt_groups = data.index//self.qt_data_queue
        if qt_groups > self.grp_sensors[sensor]:
            if sensor == self.sensor_show:
                self.trans.emit(copy.deepcopy(self.data_sensors[sensor]))
            self.trans_save.emit(sensor, copy.deepcopy(self.data_save_sensors[sensor]))
            self.data_sensors[sensor] = np.full((self.qt_var_sensor, self.qt_data_queue), 40000.0)
            self.data_save_sensors[sensor] = np.full((self.qt_info_max, self.qt_data_queue), 40000)
            self.grp_sensors[sensor] += 1
        data_needed = np.array([data.index, data.timestamp, \
                       data.ax, data.ay, data.az,\
                       data.gx, data.gy, data.gz,\
                       data.mx, data.my, data.mz])
        data_attribution = data_needed[2:(self.qt_var_sensor + 2)]
        data_converted = np.full((self.qt_var_sensor), 0.0)
        data_converted [0:3] = self.convert_data(data_needed[2:5], 'A')
        data_converted [3:6] = self.convert_data(data_needed[5:8], 'G')
        data_converted [6:9] = self.convert_data(data_needed[8:11], 'M')
        quat_result = self.kalman_filter(data_converted)
        self.quaternium.emit(quat_result)
        euler_angles = (Rot.from_quat(quat_result, scalar_first=True)).as_euler('ZYX', degrees=True)
        #self.data_sensors[sensor][:, position] = copy.deepcopy(data_converted)
        self.data_sensors[sensor][:, position] = np.concatenate((euler_angles, \
        np.array([0.0,0.0,0.0,0.0,0.0,0.0])), axis=None)
        self.data_save_sensors[sensor][:, position] = data_needed
        '''
        self.data_sensors[sensor][:, position] = data_attribution
        data_needed_converted = np.full((self.qt_var_sensor), 0.0)
        data_needed_converted [0:2] = data_needed[0:2]
        data_needed_converted [2:5] = self.convert_data(data_needed[2:5], 'A')
        data_needed_converted [5:8] = self.convert_data(data_needed[5:8], 'G')
        data_needed_converted [8:11] = self.convert_data(data_needed[8:11], 'M')
        #data_attribution = data_needed_converted[2:(self.qt_var_sensor + 2)]
        self.data_sensors_itg[sensor][:, position_itg] = data_needed_converted
        '''

    @Slot(dict)
    def trans_end_data(self, data):
        sensor = data.sensor_id
        self.trans_save.emit(sensor, copy.deepcopy(self.data_sensors[sensor]))

    @Slot(str)
    def change_ss(self, sensor):
        self.sensor_show = int(sensor)
        
    def convert_data(self, value, tp_data):
        match tp_data:
            case 'A':
                value = (((value - self.accel_blimit[0])/((self.accel_blimit[1] - self.accel_blimit[0]) + 1)) * \
                (self.accel_elimit * 2)) - self.accel_elimit 
            case 'G':
                value = (((value - self.gyr_blimit[0])/((self.gyr_blimit[1] - self.gyr_blimit[0]) + 1)) * \
                (self.gyr_elimit * 2)) - self.gyr_elimit 
            case 'M':
                value = (((value - self.mag_blimit[0])/((self.mag_blimit[1] - self.mag_blimit[0]) + 1)) * \
                (self.mag_elimit * 2)) - self.mag_elimit 
                calib_b = np.array([-15.922080,18.649750,-6.900946])
                calib_A = np.array([np.array([1.286294,0.006132,-0.005188]), \
                                    np.array([0.006132, 1.416687,0.029660]), \
                                    np.array([-0.005188, 0.029660, 1.286554])])
                value = calib_A @ (value - calib_b)
        return value

    def kalman_filter(self, data):
        Acc = copy.deepcopy(data[0:3])
        Gyr = copy.deepcopy(data[3:6])
        Mag = copy.deepcopy(data[6:9])
        '''
        #==========================================================================================
        # ROLL AND PITCH (ACC)
        #==========================================================================================
        if(Acc[2] == 0 and Acc[1] > 0):
            roll_acc = math.pi/2 * (180.0 / math.pi)
        elif(Acc[2] == 0 and Acc[1] < 0):
            roll_acc = -math.pi/2 * (180.0 / math.pi)
        elif(Acc[2] == 0 and Acc[1] == 0):
            print("Impossible case")
        else:
            roll_acc = math.atan(Acc[1]/Acc[2]) * (180.0 / math.pi)

        if (Acc[0]/FormatData.GRAV >= -1 and Acc[0]/FormatData.GRAV <= 1):
            pitch_acc = math.asin(Acc[0]/FormatData.GRAV) * (180.0 / math.pi)
        elif (Acc[0]/FormatData.GRAV > 1):
            pitch_acc = math.pi/2 * (180.0 / math.pi)
        else:
            pitch_acc = -math.pi/2 * (180.0 / math.pi)
        #==========================================================================================
        # ROLL AND PITCH (GYR)
        #==========================================================================================
        tst_R = [[1, math.sin(self.roll)*math.tan(self.pitch), math.cos(self.roll)*math.tan(self.pitch)], \
        [0, math.cos(self.roll), -math.sin(self.roll)]]
        rates = np.dot(np.array(tst_R), Gyr)
        self.roll = self.roll + (1/FormatData.FREQ) * rates[0]
        self.pitch = self.pitch + (1/FormatData.FREQ) * rates[1]
        #==========================================================================================
        # FILTRO COMPLEMENTAR
        #==========================================================================================
        alfa = 0.05
        self.roll = roll_acc * alfa + self.roll * (1-alfa)
        self.pitch = pitch_acc * alfa + self.pitch * (1-alfa)

        angles = np.array([self.roll, self.pitch, 0])
        print(angles)
        '''
        '''
        #==========================================================================================
        # TENTETIVA FILTRO DE KALMAN
        #==========================================================================================
        # DETERMINAÇÃO DA MEDIDA
        #==========================================================================================
        Zgb = Acc/np.linalg.norm(Acc)
        Xgb = np.cross((Mag/np.linalg.norm(Mag)), Zgb)
        Ygb = np.cross(Zgb, Xgb)
        #==========================================================================================
        # DETERMINAÇÃO DA MATRIZ DE ROTAÇÃO
        #==========================================================================================
        Rgb = np.array([Xgb, Ygb, Zgb])
        Y = (Rot.from_matrix(Rgb)).as_quat(scalar_first = True)
        #==========================================================================================
        # DETERMINAÇÃO DA MATRIZ DE TRANSIÇÃO DE ESTADO DO SISTEMA
        #==========================================================================================
        wx = Gyr[0]
        wy = Gyr[1]
        wz = Gyr[2]
        A = np.eye(4) + \
        (0.5*np.array([np.array([0, -wx, -wy, -wz]), \
                    np.array([wx, 0, wz, -wy]), \
                    np.array([wy, -wz, 0, wx]), \
                    np.array([wz, wy, -wx, 0])]) * (1/FormatData.FREQ))
        #==========================================================================================
        # DETERMINAÇÃO DO ESTADO DO SISTEMA
        #==========================================================================================
        self.X = np.dot(A, self.X)
        self.X = self.X/np.linalg.norm(self.X)
        #==========================================================================================
        # DETERMINAÇÃO DA MATRIZ DE COVARIÂNCIA DO ERRO
        #==========================================================================================
        self.P = np.dot(np.dot(A, self.P), np.transpose(A)) + self.Q
        #==========================================================================================
        # GANHO KALMAN
        #==========================================================================================
        K = np.dot(np.dot(self.P, np.transpose(self.C)), \
        np.linalg.inv((np.dot(np.dot(self.C, self.P), np.transpose(self.C)) + self.R)))
        #==========================================================================================
        # ATUALIZAÇÃO DA COVARIÂNCIA
        #==========================================================================================
        self.P = self.P - np.dot(K, np.dot(self.C, self.P))
        #==========================================================================================
        # ESTIMAÇÃO
        #==========================================================================================
        self.X = self.X + np.dot(K, (Y - np.dot(self.C, self.X)))
        self.X = self.X/np.linalg.norm(self.X)
        return (copy.deepcopy(self.X))
        '''
        #==========================================================================================
        # FILTRO DE KALMAN APLICADO
        #==========================================================================================
        if (self.counter == 0):
            self.Q = acc2q(Acc)
        self.Q = self.ekf.update(q = self.Q, gyr = (Gyr * (math.pi/180)), acc = Acc, mag = Mag) 
        return copy.deepcopy(self.Q)

class CentralCtrl(QObject):
    list_sensor = pyqtSignal(tuple)
    update_sensors = pyqtSignal(dict)
    LIVE_LIST_TIMEOUT_S = 5
    MAX_DEVICES = 15
    def __init__(self):
        super().__init__()
        self.lock = QMutex()
        self.live_list = {}

    @Slot(tuple)
    def get_config_rxq(self, config_rxq):
        (ip,port,data) = config_rxq
        data = commands.decode(data)
        if data:
            self.lock.lock()
            self.live_list_add(ip,port,data)
            self.lock.unlock()

    def live_list_add(self,ip,port,data):
        if data['id'] not in self.live_list:
            print(f"Sensor {data['id']} added")
            self.list_sensor.emit((data['id'], ip))
            self.live_list[data['id']] = {}
            self.live_list[data['id']]['event'] = True
            self.live_list[data['id']]['task'] = True
        self.live_list[data['id']]['last_seen'] = util.time()
        self.live_list[data['id']]['id'] = data['id']
        self.live_list[data['id']]['state'] = data['state']
        self.live_list[data['id']]['rate'] = data['rate']
        self.live_list[data['id']]['axis'] = data['axis']
        self.live_list[data['id']]['cnt'] = data['cnt']
        self.live_list[data['id']]['ip'] = ip
        self.live_list[data['id']]['port'] = port

    @Slot(tuple)
    def client_connect(self, params):
        (id, status) = params
        self.live_list[id]['task'] = status

    def get_sensor_list(self):
        lvl = {}
        for k,v in self.live_list.items():
            lvl[k] = { 'rate':v['rate'], 'axis':v['axis'], 'ip':v['ip'], 'port':v['port'], 'id': v['id'],
                      'state':v['state'], 'cnt':v['cnt']}
        return lvl

    def live_list_update(self):
        to_remove = []
        for k,v in self.live_list.items():
            if (util.time() - v['last_seen']) > CentralCtrl.LIVE_LIST_TIMEOUT_S or v['task'] == False:
                print("Conexão dados remove:", v['task'])
                to_remove.append(k)
        
        for sensor_id in to_remove:
            self.live_list[sensor_id]['event'] = False
            print(f"Sensor {sensor_id} removed")
            del self.live_list[sensor_id]

    def check_queues(self):
        if self.live_list:
            self.lock.lock()
            self.live_list_update()
            self.update_sensors.emit(self.get_sensor_list())
            self.lock.unlock()

    @Slot(tuple)
    def send_config(self,params):
        (sensor_id,state,rate,axis) = params
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(get_default_ip()))
        data = commands.encode(commands.CMD_ANNOUCE,sensor_id=sensor_id,state=state,rate=rate,axis=axis,cnt=0)
        if data:
            sock.sendto(data,(SEN_CFG_MC_IP,PORT))

    def close(self):
        for k,v in self.live_list.items():
            print(f'Signalizing client task for sensor id {k}')
            v['event'] = False
        try:
            self.config_server.close()
        except:
            pass
        print('waiting config server')
        self.config_server.join()

class CentralCtrlConfigServer(QObject):
    config_rxq = pyqtSignal(tuple)
    def __init__ (self):
        super().__init__()
        self.stop = False
    
    def run(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(2)
        try:
            self.sock.bind(('', PORT))
            itf = get_default_ip()
            print(f'Default interface for MC: {itf}')
            group = socket.inet_aton(SVR_CFG_MC_IP)
            iface = socket.inet_aton(itf)
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, group+iface)
        except:
            print('Can not bind to config MC port')
            return
        while not self.stop:
            try:
                # Recvfrom retorna o par (bytes, adress). Adress é o endereço do soquete que está 
                # recebendo os dados. Ele é do formato (host, port), sendo host um endereço IPv4 
                # em string e port um inteiro
                (data,client) = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            except:
                self.sock = None
                break
            if not data:
                break
            self.config_rxq.emit((client[0],client[1],data))
    
        if self.sock:
            group = socket.inet_aton(SVR_CFG_MC_IP)
            iface = socket.inet_aton(get_default_ip())
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, group+iface)

    def close(self):
        if self.sock:
            self.sock.close()

    def close_loop(self, status):
        self.stop = status

class CentralCtrlSensorClient(QObject):
    connected = pyqtSignal(tuple)
    decoded_data = pyqtSignal(object)
    def __init__ (self, id_ip):
        super().__init__()
        (self.sensor_id, self.ip) = id_ip
        self.port = self.sensor_id + PORT
        self.stop = False
        print(f"Begin class SensorClient for sensor {self.sensor_id}")

    def run(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, True)
        frmax_sensor = 500 # Frequência máxima de aquisição de cada sensor
        nsensors = 10 # Quantidade máxima de sensores na rede
        bytes_sensor = frmax_sensor*commands.CMD_SIZES[commands.CMD_DATA_AGM]
        rx = bytes_sensor*nsensors
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,rx) 
        bufsize = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF) 
        self.sock.settimeout(1)
        try:
            self.sock.connect((self.ip, self.port))
        except:
            self.connected.emit((self.sensor_id, False))
            print(f'Can not connect to sensor {self.ip}:{self.port}')
            return

        print(f'Connect to sensor on {self.ip}:{self.port}')
        self.buffer = b''
        self.connected.emit((self.sensor_id, True))
        while not self.stop:
            try:
                data = self.sock.recv(14500)
            except socket.timeout:
                continue
            except KeyboardInterrupt:
                self.sock = None
                break
            except:
                self.sock = None
                break

            if not data:
                break
            
            if data:
                self.dispatch_data(data) # Recebe dados também para manter o sensor 'vivo'

        print(f'Closing data client for {self.ip}:{self.port}')
        self.connected.emit((self.sensor_id, False))
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
            self.sock.close()
        except:
            pass
        
    def dispatch_data(self,data):
        self.buffer = self.buffer + data
        pos = 0
        full_size = len(self.buffer)
        while pos < full_size:
            cmd_id = self.buffer[pos]
            if (pos + commands.CMD_SIZES[cmd_id]) <= full_size:
                if cmd_id in [commands.CMD_DATA_A,commands.CMD_DATA_AG,commands.CMD_DATA_AGM]:
                    #TEMPO_END = time.perf_counter_ns()
                    #print(TEMPO_END)
                    #global TEMPO
                    #print("Start time:", TEMPO, "End time:", TEMPO_END, "Subtração:", TEMPO_END-TEMPO)
                    decoded = commands.decode(self.buffer[pos:pos+commands.CMD_SIZES[cmd_id]])
                    #print(decoded)
                    if decoded:
                        # Para verificar em que Thread está: QThread.currentThread()
                        self.decoded_data.emit(copy.deepcopy(decoded['data']))
                pos += commands.CMD_SIZES[cmd_id]
            else:
                break
        self.buffer = self.buffer[pos:]
    
    def close_loop(self, status):
        self.stop = status

class MainGui(QtWidgets.QMainWindow):
    close_all = pyqtSignal(bool)
    start_stop_data = pyqtSignal(tuple)
    sensor_axis = pyqtSignal(int)
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        self.ui = uic.loadUi(resource_path('mainpqg.ui'),self)
        #==========================================================================================
        # DICIONÁRIO GERAL PARA AS THREADS SENSOR CLIENT
        #==========================================================================================
        self.thread_sensor_client = {}
        self.sensor_client = {}
        #==========================================================================================
        # CRIAÇÃO DA THREAD DO CENTRAL CTRL
        #==========================================================================================
        # OBS.: É importante fazer a thread não daemon, para que o processo não seja interrompido
        # quando o programa principal chega ao final
        #==========================================================================================
        self.thread_central_ctrl = QThread()
        self.central_ctrl = CentralCtrl()
        self.central_ctrl.moveToThread(self.thread_central_ctrl)
        #==========================================================================================
        # CRIAÇÃO DA THREAD DO CONFIG SERVER (PARA PASSAR O SINAL DA CONFIG_RXQ)
        #==========================================================================================
        self.thread_config_server = QThread()
        self.config_server = CentralCtrlConfigServer()
        self.config_server.moveToThread(self.thread_config_server)
        self.thread_config_server.started.connect(self.config_server.run)
        #==========================================================================================
        # CONEXÃO ENTRE OS SIGNALS E OS SLOTS
        #==========================================================================================
        self.config_server.config_rxq.connect(self.central_ctrl.get_config_rxq)
        self.central_ctrl.list_sensor.connect(self.create_sensor_client)
        self.central_ctrl.update_sensors.connect(self.gui_update_sensors)
        self.close_all.connect(self.config_server.close_loop)
        self.close_all.connect(self.config_server.close)
        self.start_stop_data.connect(self.central_ctrl.send_config)
        #==========================================================================================
        # START DAS THREADS CRIADAS
        #==========================================================================================
        self.thread_central_ctrl.start()
        self.thread_config_server.start()
        #==========================================================================================
        # CRIAÇÃO DA THREAD PARA A FORMATAÇÃO DOS DADOS
        #==========================================================================================
        self.thread_frtdata = QThread()
        self.frtdata = FormatData()
        self.frtdata.moveToThread(self.thread_frtdata)
        self.frtdata.trans.connect(self.putpdata)
        self.sensor_axis.connect(self.frtdata.get_axis_id)
        self.thread_frtdata.start()
        #==========================================================================================
        # FLAG PARA VERIFICAR SE A COLETA DE DADOS FOI FEITA AO MENOS UMA VEZ
        #==========================================================================================
        self.flag_start = False
        #==========================================================================================
        # PARÂMETROS DE TAXA E EIXOS
        #==========================================================================================
        (self.rate, self.rate_id) = (500, 0)
        (self.axis, self.axis_id) = ('AGM', 2)
        self.window_length = 1000
        #==========================================================================================
        # CRIAÇÃO DOS GRÁFICOS
        #==========================================================================================
        self.set_graphs_params()
        self.define_graphs()
        self.define_space()
        #==========================================================================================
        # DEFINIÇÃO DO UPDATE DOS GRÁFICOS
        #==========================================================================================
        self.update_plot()
        self.timer_graph = QtCore.QTimer()
        self.timer_graph.setInterval(20) # Intervalo em ms
        self.timer_graph.timeout.connect(self.update_plot)
        self.timer_graph.start()
        #==========================================================================================
        # DEFINIÇÃO DO UPDATE DA AQUISIÇÃO DOS SENSORES
        #==========================================================================================
        self.timer_update_sensors = QtCore.QTimer()
        self.timer_update_sensors.setInterval(1000) # Intervalo em ms
        self.timer_update_sensors.timeout.connect(self.central_ctrl.check_queues)
        self.timer_update_sensors.start()
        #==========================================================================================
        # RELAÇÃO ENTRE OS ELEMENTOS PRESENTES NA JANELA E SUAS FUNÇÕES
        #==========================================================================================
        self.lineEditWl.textChanged['QString'].connect(self.update_window_length)
        self.comboBoxSr.currentTextChanged['QString'].connect(self.update_rate)
        self.comboBoxAx.currentTextChanged['QString'].connect(self.update_axis)
        self.comboBoxIdg.currentTextChanged['QString'].connect(self.frtdata.change_ss)
        self.pushButtonStart.clicked.connect(self.start_stream)   
        self.pushButtonStart.clicked.connect(self.frtdata.define_storage_data)  
        self.pushButtonStop.clicked.connect(self.stop_stream)  
    
    def set_graphs_params(self):
        self.y_limits = [self.frtdata.accel_blimit,self.frtdata.accel_blimit, self.frtdata.accel_blimit, 
                         self.frtdata.gyr_blimit,self.frtdata.gyr_blimit, self.frtdata.gyr_blimit,
                         self.frtdata.mag_blimit, self.frtdata.mag_blimit, self.frtdata.mag_blimit]
        self.positions_rc = [(0, 0), (0, 1), (0, 2), 
                             (1, 0), (1, 1), (1, 2),
                             (2, 0), (2, 1), (2, 2)]
        self.title_graphs = ["Ax", "Ay", "Az",
                             "Gx", "Gy", "Gz", 
                             "Mx", "My", "Mz"]

    
    def define_graphs(self):
        qt_graphs = (self.axis_id + 1) * 3

        try:
            self.graphicsView.clear()
        except:
            pass

        self.graphs = np.empty(qt_graphs, dtype = object) # 9 gráficos a serem criados
                                                          # a(x, y, z), g(x, y, z), m(x, y, z)
        self.curves = np.empty(qt_graphs, dtype = object)

        for i in range(qt_graphs):
            self.graphs[i] = self.graphicsView.addPlot(row = self.positions_rc[i][0], 
                                                       col = self.positions_rc[i][1], 
                                                       title = self.title_graphs[i],
                                                       pen = pg.mkPen(color='#00ff00', width=2))
            self.curves[i] = self.graphs[i].plot()
            self.graphs[i].setRange(yRange = self.y_limits[i], disableAutoRange=True)

        self.pdata = np.zeros((qt_graphs,1))
        length  = int(self.window_length*self.rate/1000)
        self.plotdata = np.zeros((qt_graphs, length))

    def define_space(self):
        grid = gl.GLGridItem()
        axis = gl.GLAxisItem()
        self.openGLWidget.addItem(axis)
        self.openGLWidget.addItem(grid)
        #======================================================================================
        # DESENHO DO CUBO
        #======================================================================================
        vertexes = np.array([[0.5, -0.5, -0.5], #0
                            [-0.5, -0.5, -0.5], #1
                            [-0.5, 0.5, -0.5], #2
                            [-0.5, -0.5, 0.5], #3
                            [0.5, 0.5, -0.5], #4
                            [0.5, 0.5, 0.5], #5
                            [-0.5, 0.5, 0.5], #6
                            [0.5, -0.5, 0.5]])#7
        faces = np.array([[1,0,7], [1,3,7],
                  [1,2,4], [1,0,4],
                  [1,2,6], [1,3,6],
                  [0,4,5], [0,7,5],
                  [2,4,5], [2,6,5],
                  [3,6,5], [3,7,5]])
        colors = np.array([[1,1/(1+i*10),1/(1+i*10),1] for i in range(12)])
        self.cube = gl.GLMeshItem(vertexes=vertexes, faces=faces, faceColors=colors,
                     drawEdges=True, edgeColor=(0, 0, 0, 1))
        self.openGLWidget.addItem(self.cube)
    #==========================================================================================
    # CRIAÇÃO E CONEXÃO DAS THREADS SENSOR CLIENTS
    #==========================================================================================
    @Slot(tuple)
    def create_sensor_client(self, params):
        id = params[0]
        self.thread_sensor_client[id] = QThread()
        self.sensor_client[id] = CentralCtrlSensorClient(params)
        self.sensor_client[id].connected.connect(self.central_ctrl.client_connect)
        self.sensor_client[id].decoded_data.connect(self.frtdata.join_data)
        self.close_all.connect(self.sensor_client[id].close_loop)
        self.sensor_client[id].moveToThread(self.thread_sensor_client[id])
        self.thread_sensor_client[id].started.connect(self.sensor_client[id].run)
        self.thread_sensor_client[id].start()
        print("Objeto cliente", id, "associado:", self.sensor_client[id].thread())

    @Slot(dict)
    def gui_update_sensors(self, list_sensors):
        sensors = list(list_sensors.keys())
        qt_sensors = len(list_sensors.keys())
        self.tableWidget.setRowCount(qt_sensors)
        for row in range(qt_sensors):
            data_sensor = list_sensors[sensors[row]]
            self.tableWidget.setItem(row, 0, QtWidgets.QTableWidgetItem(str(data_sensor['ip'])))
            self.tableWidget.setItem(row, 1, QtWidgets.QTableWidgetItem(str(data_sensor['id'])))
            self.tableWidget.setItem(row, 2, QtWidgets.QTableWidgetItem(str(data_sensor['cnt'])))

    def start_worker(self):
        self.thread_save_data = QThread()
        self.save_data = SaveData()
        self.save_data.moveToThread(self.thread_save_data)
        self.frtdata.trans_save.connect(self.save_data.update)
        self.frtdata.quaternium.connect(self.update_cube_angle)
        self.pushButtonStop.clicked.connect(self.save_data.close_file) 
        self.thread_save_data.start()
        self.flag_start = True

    def putpdata(self, value):
        self.pdata = value
        for i in range(len(self.curves)):
            shift = len(self.pdata[i])
            self.plotdata[i] = np.roll(self.plotdata[i], -shift)
            self.plotdata[i, -shift:] = self.pdata[i]

    def start_stream(self):
        self.enable_disable_widgets(False)
        #==========================================================================================
        # ENVIO DOS PARÂMETROS PARA O INÍCIO DA AQUISIÇÃO DE DADOS
        # O primeiro dado se refere ao id do sensor (no caso 0xFF está relacionado com todos
        # os sensores); o segundo ao estado ('INIT', 'HOLD' e 'RUN'); o terceiro à taxa de 
        # transmissão ('500Hz','400Hz','300Hz','200Hz','100Hz','1Hz'); o quarto aos dados
        # ('A', 'AG', 'AGM')
        #==========================================================================================
        state_id = self.get_id_state('RUN')
        self.start_stop_data.emit((0xFF, state_id, self.rate_id, self.axis_id))
        self.start_worker()

    def stop_stream(self):
        self.enable_disable_widgets(True)
        state_id = self.get_id_state('HOLD')
        self.start_stop_data.emit((0xFF, state_id, self.rate_id, self.axis_id))
        self.thread_save_data.exit()

    def enable_disable_widgets(self, flag):
        self.lineEditWl.setEnabled(flag)
        self.comboBoxSr.setEnabled(flag)
        self.comboBoxAx.setEnabled(flag)
        self.pushButtonStart.setEnabled(flag)

    def update_window_length(self,value):
        self.window_length = int(value)

    def update_rate(self,rate):
        self.rate = int(rate)
        self.rate_id = list(commands.RATES.keys())[list(commands.RATES.values()).index(rate)]

    def update_axis(self, axis):
        self.axis = axis
        self.axis_id = list(commands.AXIS.keys())[list(commands.AXIS.values()).index(axis)]
        self.define_graphs()
        self.sensor_axis.emit(self.axis_id)

    def get_id_state(self, state):
        return(list(commands.STATES.keys())[list(commands.STATES.values()).index(state)])

    
    def update_cube_angle(self, quat):
        #q = QtGui.QQuaternion(quat[0], quat[1], quat[2], quat[3])
        #q = QtGui.QQuaternion(1,0,1,0).normalized()
        #axis, angle = q.getAxisAndAngle()
        #self.cube.rotate(2*np.arccos(quat[0]), quat[1], quat[2], quat[3])
        self.cube.resetTransform()
        #self.cube.rotate(90, 1, 0, 0)
        #self.cube.rotate(360, 0, 0, 1)
        angle = math.acos(quat[0]) * 2 * ( 180.0/math.pi)
        angle_rad = math.acos(quat[0]) * 2
        self.cube.rotate(angle, quat[1]/math.sin(angle_rad/2), quat[2]/math.sin(angle_rad/2), quat[3]/math.sin(angle_rad/2))

    def update_plot(self):
        try:
            for i in range(len(self.curves)):
                self.curves[i].setData(self.plotdata[i])
        except:
            pass
    
    def closeEvent(self, event):
        self.close_all.emit(True)
        self.thread_central_ctrl.exit()
        self.thread_config_server.exit()
        self.thread_frtdata.exit()
        for key in self.sensor_client.keys():
            self.thread_sensor_client[key].exit()

def main():
    app = QtWidgets.QApplication(sys.argv)
    mainWindow = MainGui()
    mainWindow.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()

