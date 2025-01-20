from queue import Queue, PriorityQueue
from imu import ImuData, QDATA_NEW, QDATA_LOST
import copy

class DataHandler():
    def __init__(self):
        self.missed_packets = 0
        self.total_frames = 0
        self.sensor_packet_counter = {0:0}
        
    def init(self,sensor_id_list):
        self.expected_index = 0
        self.sensor_id_list = sensor_id_list
        self.missed_packets = 0
        self.total_frames = 0
        self.sensor_packet_counter = dict(zip(self.sensor_id_list,[ 0 for n in range(len(sensor_id_list))]))
        self.started = dict(zip(self.sensor_id_list,[ False for n in range(len(sensor_id_list))]))
        self.data_frame = dict(zip(self.sensor_id_list,[ ImuData(sensor_id=n) for n in range(len(sensor_id_list))]))
        self.input_queues = dict(zip(self.sensor_id_list,[ Queue() for n in range(len(sensor_id_list))]))

    def put(self,imu):
        # only selected sensors
        if imu.sensor_id not in self.sensor_id_list:
            return
        
        # await the first data package, drop older ones (if any)
        if imu.index == 0:
            self.started[imu.sensor_id] = True

        if self.started[imu.sensor_id]:
            self.input_queues[imu.sensor_id].put(imu)
            self.sensor_packet_counter[imu.sensor_id] += 1
    
    def get_stats(self):
        total_packets = sum([ x for x in self.sensor_packet_counter.values() ])
        return (self.total_frames,total_packets,self.missed_packets)

    def get(self):
        # check if we have at least one imu data per sensor
        frame_completed = True
        for queue in self.input_queues.values():
            if queue.empty():
                frame_completed = False
                break
        
        if not frame_completed:
            return {}
        
        idx = self.expected_index
        for sid,queue in self.input_queues.items():
            imu = queue.get(False)
            if idx == imu.index:
                # current index awaited !
                self.data_frame[sid] = imu
                self.data_frame[sid].quality = QDATA_NEW
            elif imu.index > idx:
                # future item (something lost?)... put it back and use last value (already saved into data_frame)
                self.input_queues[sid].put(imu)
                self.data_frame[sid].quality = QDATA_LOST
                self.data_frame[sid].index = idx
                self.missed_packets += 1

        self.total_frames += 1
        self.expected_index += 1

        return copy.deepcopy(self.data_frame)
