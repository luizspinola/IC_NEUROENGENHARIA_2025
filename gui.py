from PyQt6 import QtCore, QtWidgets
from PyQt6 import uic
from PyQt6.QtCore import QThread, pyqtSlot as Slot, pyqtSignal
from utils import resource_path
from communications import CentralCtrl, CentralCtrlConfigServer, CentralCtrlSensorClient
from dataprocessing import FormatData, SaveData
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import commands
import math

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
        colors = np.array([[1/(1+i*10),1,1/(1+i*10),1] for i in range(12)])
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
        self.frtdata.displacement.connect(self.update_cube_position)
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
        self.cube.resetTransform()
        angle = math.acos(quat[0]) * 2 * ( 180.0/math.pi)
        angle_rad = math.acos(quat[0]) * 2
        self.cube.rotate(angle, quat[1]/math.sin(angle_rad/2), quat[2]/math.sin(angle_rad/2), quat[3]/math.sin(angle_rad/2))

    def update_cube_position(self, pos):
        #self.cube.resetTransform()
        print("Posição recebida:", pos)
        self.cube.translate(pos[0], pos[1], pos[2])
    
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