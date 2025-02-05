from PyQt6.QtCore import QObject, pyqtSlot as Slot, pyqtSignal, QMutex
import utils as util
import socket
import commands
import copy

SVR_CFG_MC_IP = '230.0.0.1'
SEN_CFG_MC_IP = '230.0.0.2'
PORT = 50000
from utils import get_default_ip

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
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2) #Teria que modificar 2 para id pra mudar o canal pra cada sensor? 
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
        self.connected.emit((self.sensor_id, False)) #é daí que a interface fecha ao ligar mais de um sensor?
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
