import struct
from imu import ImuData

CMD_ANNOUCE = 0
CMD_DATA_A = 1
CMD_DATA_AG = 2
CMD_DATA_AGM = 3
CMD_KEEP_ALIVE = 4
CMD_LATENCE = 5

STATE_IDS = [0, 1, 2]
STATE_NAMES = ['INIT', 'HOLD', 'RUN']
STATES = dict(zip(STATE_IDS,STATE_NAMES))
SENSOR_IDS = [0, 1, 2, 3, 4, 5]
SENSOR_NAMES= [1, 2, 3, 4, 5, 6]
AXIS_IDS = [0, 1, 2]
AXIS_NAMES = ['A', 'AG', 'AGM']
AXIS = dict(zip(AXIS_IDS,AXIS_NAMES))
RATE_IDS = [0,1,2,3,4,5]
RATE_NAMES = ['500','400','300','200','100','1']
RATES = dict(zip(RATE_IDS,RATE_NAMES))
TIME_IDS = [0, 1, 2, 3, 4]
TIME_NAMES = ['10s', '20s', '30s', '40s', '60s']
TIMES = dict(zip(TIME_IDS,TIME_NAMES))
POSITION_IDS = [0, 1, 2, 3]
POSITION_NAMES = ['Bíceps (S)', 'Bíceps (I)', 'Braquiorradial (I)', 'Supraespinhal (D)']
POSITIONS = dict(zip(POSITION_IDS,POSITION_NAMES))
CMD_SIZES = {CMD_ANNOUCE: 9, CMD_DATA_A:17, CMD_DATA_AG:23, CMD_DATA_AGM:29, CMD_KEEP_ALIVE:1, CMD_LATENCE: 1}


def axis_label_is_valid(axis_lab):
    return axis_lab in AXIS_NAMES
    
def rate_hz_is_valid(rate_hz):
    return rate_hx_idx_conv(rate_hz) in RATE_IDS

def rate_hx_idx_conv(hz):
    if hz == 1:
        return 5
    else:
        return 5 - hz/100

def rate_idx_hz_conv(idx):
    if idx == 5:
        return 1
    else:
        return (5-idx)*100

def axis_idx_num_axis(idx):
    return (idx+1)*3

def decode(data):
    cmd = {}
    if len(data) >= 1:
        if CMD_ANNOUCE == data[0] and len(data) == CMD_SIZES[CMD_ANNOUCE]:
            (cmd_id,sensor_id,state,rate,axis,cnt) = struct.unpack('>BBBBBL',data)
            cmd['cmd'] = cmd_id
            cmd['id'] = sensor_id
            cmd['state'] = state
            cmd['rate'] = rate
            cmd['axis'] = axis
            cmd['cnt'] = cnt
        elif CMD_DATA_A == data[0] and len(data) == CMD_SIZES[CMD_DATA_A]:
            (cmd_id,sensor_id,state,index,timestamp,ax,ay,az) = struct.unpack('>BBBLLhhh',data)
            cmd['cmd'] = cmd_id
            cmd['state'] = state
            cmd['data'] = ImuData(sensor_id,index,timestamp,ax,ay,az)
        elif CMD_DATA_AG == data[0] and len(data) == CMD_SIZES[CMD_DATA_AG]:
            (cmd_id,sensor_id,state,index,timestamp,ax,ay,az,gx,gy,gz) = struct.unpack('>BBBLLhhhhhh',data)
            cmd['cmd'] = cmd_id
            cmd['state'] = state
            cmd['data'] = ImuData(sensor_id,index,timestamp,ax,ay,az,gx,gy,gz)
        elif CMD_DATA_AGM == data[0] and len(data) == CMD_SIZES[CMD_DATA_AGM]:
            (cmd_id,sensor_id,state,index,timestamp,ax,ay,az,gx,gy,gz,mx,my,mz)= struct.unpack('>BBBLLhhhhhhhhh',data)
            cmd['cmd'] = cmd_id
            cmd['state'] = state
            cmd['data'] = ImuData(sensor_id,index,timestamp,ax,ay,az,gx,gy,gz,mx,my,mz)
        elif CMD_KEEP_ALIVE == data[0]  and len(data) == CMD_SIZES[CMD_KEEP_ALIVE]:
            (cmd_id,)= struct.unpack('>B',data)
            cmd['cmd'] = cmd_id
        elif CMD_LATENCE == data[0] and len(data) == CMD_SIZES[CMD_LATENCE]:
            (cmd_id,)= struct.unpack('>B',data)
            cmd['cmd'] = cmd_id
        
    return cmd

def encode(cmd_id,**args):
    data = ''
    if cmd_id == CMD_ANNOUCE:
        data = struct.pack('>BBBBBL',
                            CMD_ANNOUCE,
                            args['sensor_id'],
                            args['state'],
                            args['rate'],
                            args['axis'],
                            args['cnt'])
    elif cmd_id == CMD_DATA_A:
        data = struct.pack('>BBBLLhhh',
                            CMD_DATA_A,
                            args['sensor_id'],
                            args['state'],
                            args['index'],
                            args['timestamp'],
                            args['ax'],args['ay'],args['az'])
    elif cmd_id == CMD_DATA_AG:
        data = struct.pack('>BBBLLhhhhhh',
                            CMD_DATA_AG,
                            args['sensor_id'],
                            args['state'],
                            args['index'],
                            args['timestamp'],
                            args['ax'],args['ay'],args['az'],
                            args['gx'],args['gy'],args['gz'])
    elif cmd_id == CMD_DATA_AGM:
        data = struct.pack('>BBBLLhhhhhhhhh',
                            CMD_DATA_AGM,
                            args['sensor_id'],
                            args['state'],
                            args['index'],
                            args['timestamp'],
                            args['ax'],args['ay'],args['az'],
                            args['gx'],args['gy'],args['gz'],
                            args['mx'],args['my'],args['mz'])
    elif cmd_id == CMD_KEEP_ALIVE:
        data = struct.pack('>B',CMD_KEEP_ALIVE)
    elif cmd_id == CMD_LATENCE:
        data = struct.pack('>B',CMD_LATENCE)

    return data
    