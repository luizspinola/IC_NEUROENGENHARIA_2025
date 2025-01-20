from functools import total_ordering

class ImuSimul():
    def __init__(self):
        self.ax = 0
        self.ay = 0
        self.az = 0
        self.gx = -50
        self.gy = -50
        self.gz = -50
        self.mx = +50
        self.my = +50
        self.mz = +50
        self.counter = 0

    def get(self):
        self.counter += 1
        self.ax += 1
        self.ay += 1
        self.az += 1
        self.gx += 1
        self.gy += 1
        self.gz += 1
        self.mx -= 1
        self.my -= 1
        self.mz -= 1
        if self.counter > 200:
            self.ax = self.ax = self.az = 0
            self.gx = self.gx = self.gz = -50
            self.mx = self.mx = self.mz = +50
            self.counter = 0

        return(self.ax,self.ay,self.az,self.gx,self.gy,self.gz,self.mx,self.my,self.mz)

QDATA_NEW = 0
QDATA_LOST = 1

@total_ordering
class ImuData():
    def __init__(self,sensor_id,index=0,timestamp=0,ax=0,ay=0,az=0,gx=0,gy=0,gz=0,mx=0,my=0,mz=0):
        self.sensor_id = sensor_id
        self.index = index
        self.timestamp = timestamp
        self.quality = 0
        self.ax = ax
        self.ay = ay
        self.az = az
        self.gx = gx
        self.gy = gy
        self.gz = gz
        self.mx = mx
        self.my = my
        self.mz = mz

    def __eq__(self, other):
        if not isinstance(other, __class__):
            return NotImplemented
        return self.index == other.index

    def __lt__(self, other):
        if not isinstance(other, __class__):
            return NotImplemented
        return self.index < other.index
    
    def __repr__(self) -> str:
        return f'{self.ax},{self.ay},{self.az},{self.gx},{self.gy},{self.gz},{self.mx},{self.my},{self.mz}'
    
    def get_dict(self):
        d = {
                'ax':int(self.ax),'ay':int(self.ay),'az':int(self.az),
                'gx':int(self.gx),'gy':int(self.gy),'gz':int(self.gz),
                'mx':int(self.mx),'my':int(self.my),'mz':int(self.mz),
                'index':int(self.index), 'timestamp':int(self.timestamp),
                'sensor_id':int(self.sensor_id),'quality':int(self.quality)
            }
        return d
        