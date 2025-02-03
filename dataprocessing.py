import numpy as np
from PyQt6.QtCore import QObject, pyqtSlot as Slot, pyqtSignal
import copy
from ahrs.filters import Madgwick
from ahrs.common.orientation import acc2q
from scipy.spatial.transform import Rotation as Rot
import math

class SaveData(QObject):
    def __init__(self):
        super().__init__()
        self.file = open("dados.imu", "wb")

    @Slot(int, np.ndarray)
    def update(self, sensor, data):
        self.file.write(sensor.to_bytes(4, 'big') + np.ndarray.tobytes(data))

    def close_file(self):
        self.file.close()
        
class FormatData(QObject):
    trans = pyqtSignal(np.ndarray)
    trans_save = pyqtSignal(int, np.ndarray)
    quaternium = pyqtSignal(np.ndarray)
    displacement = pyqtSignal(np.ndarray)
    # Valor válido para todas as instâncias dessa classe
    AFS_SEL = 0
    FS_SEL = 0
    FREQ = 500
    GRAV = 9.80665
    DT = 1/FREQ

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

        #self.define_var_kalman()

        self.roll = 0
        self.pitch = 0
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])

        #Definição de variáveis para o filtro de Kalman extendido
        self.Madgwick = Madgwick(frequency = 500.0)
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

    '''def define_var_kalman(self):
         self.X = np.array([1.0, 0.0, 0.0, 0.0])
         self.P = np.eye(4)
         self.R = np.eye(4) * 10.0
         self.Q = np.eye(4) * 1e-4
         self.C = np.eye(4)
    '''
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
        quat_result = self.madgwick_filter(data_converted)
        rot_matrix = Rot.from_quat(quat_result).as_matrix() #Matriz de rotação do quaternion
        acc_global = rot_matrix @ (data_converted[0:3] - np.array([0.0, 0.0, float(FormatData.GRAV)])) #Referencial Global
        self.velocity += acc_global * FormatData.DT
        self.position += self.velocity * FormatData.DT
        self.displacement.emit(self.position)
        self.quaternium.emit(quat_result)
        #euler_angles = (Rot.from_quat(quat_result, scalar_first=True)).as_euler('ZYX', degrees=True)
        #self.data_sensors[sensor][:, position] = copy.deepcopy(data_converted)
        #self.data_sensors[sensor][:, position] = np.concatenate((euler_angles, \
        #np.array([0.0,0.0,0.0,0.0,0.0,0.0])), axis=None)
        #self.data_save_sensors[sensor][:, position] = data_needed
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

    def madgwick_filter(self, data):
        Acc = copy.deepcopy(data[0:3])
        Gyr = copy.deepcopy(data[3:6])
        Mag = copy.deepcopy(data[6:9])
       
        if (self.counter == 0):
            self.Q = acc2q(Acc)
        self.Q = self.Madgwick.updateMARG(q = self.Q, gyr = (Gyr * (math.pi/180)), acc = Acc, mag = Mag) 
        return copy.deepcopy(self.Q)


