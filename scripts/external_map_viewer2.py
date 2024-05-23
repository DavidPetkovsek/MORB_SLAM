import asyncio
import cv2
import websockets
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread, Lock

def parse_binary_data(binary_data):
        isFromSLAM = binary_data[0]
        if isFromSLAM==1:
            # rotation_matrix = np.frombuffer(binary_data[1:37], dtype=np.float32).reshape(3, 3)
            
            translation_vector = np.frombuffer(binary_data[37:49], dtype=np.float32)
            # state = struct.unpack('<i', binary_data[49:53])[0]
            # message = struct.unpack('<i', binary_data[53:57])[0]
            # isKF = struct.unpack('<?', binary_data[57:58])[0]
            # return isFromSLAM, rotation_matrix, translation_vector, state, message, isKF
            return isFromSLAM, translation_vector
        else:
            translation_vector = np.frombuffer(binary_data[1:13], dtype=np.float32)
            # return isFromSLAM, None, translation_vector, None, None, None
            return isFromSLAM, translation_vector

async def main():
    uri = "ws://192.168.1.1:9002"

    try:
        async with websockets.connect(uri, ping_interval=None) as websocket:
            n = 0
            n2 = 0
            c = 3
            while not shouldClose:
                # async with asyncio.timeout(timeout=1.5):
                binary_data = await websocket.recv()
                isFromSLAM, translation_vector = parse_binary_data(binary_data)
                n += 1
                n2 += 1
                if isFromSLAM:
                    if n > c:
                        if p_SLAM.update(translation_vector[0], translation_vector[1], False):
                            n=0
                else:
                    if n2 > c:
                        if p_AIV.update(translation_vector[0], translation_vector[1], False):
                            n2=0
                
    except:
        import traceback
        traceback.print_exc()
        

def minMax(minX, maxX, minY, maxY, padding=0.1, numTicks=10):
    dx = maxX-minX
    dy = maxY-minY
    scaleAmount = max(dx,dy)*padding

    if dx > dy:
        # use X as base
        avg = (maxY+minY)/2
        rangeX = np.linspace(minX-scaleAmount, maxX+scaleAmount, numTicks)
        rangeY = np.linspace((avg-dx/2)-scaleAmount, (avg+dx/2)+scaleAmount, numTicks)
    else:
        # use Y as base
        avg = (maxX+minX)/2
        rangeY = np.linspace(minY-scaleAmount, maxY+scaleAmount, numTicks)
        rangeX = np.linspace((avg-dy/2)-scaleAmount, (avg+dy/2)+scaleAmount, numTicks)
    
    return rangeX, rangeY

class fancy_matplot:
    def __init__(self, name=None):
        self.fig = plt.figure(figsize=(8,8))
        self.ax = self.fig.add_subplot(111)
        self.Ln, = self.ax.plot([0,0])
        self.datX=[]
        self.datY=[]
        self.lock = Lock()
        if not name is None:
            self.ax.set_title(name)
        self.minX = float('inf')
        self.maxX = -float('inf')
        self.minY = float('inf')
        self.maxY = -float('inf')
        self.updated = False

    def update(self, valX, valY, doDraw=True):
        updated = False
        with self.lock:
            if valX < self.minX: self.minX = valX
            if valY < self.minY: self.minY = valY
            if valX > self.maxX: self.maxX = valX
            if valY > self.maxY: self.maxY = valY
            # print(np.sqrt((valX-self.datX[-1])**2 + (valY-self.datY[-1])**2))
            if len(self.datX) < 4 or len(self.datY) < 4 or (not (np.sqrt((valX-self.datX[-1])**2 + (valY-self.datY[-1])**2) < 0.0001)):
                self.datX.append(valX)
                self.datY.append(valY)
                self.updated = True
                updated = True
        if doDraw:
            self.draw()
        return updated

    def draw(self):

        with self.lock:

            if self.updated:
                
                self.Ln.set_ydata(self.datY)
                self.Ln.set_xdata(self.datX)
                xr, yr = minMax(self.minX, self.maxX, self.minY, self.maxY)
                self.ax.set_xticks(xr)
                self.ax.set_yticks(yr)
                self.fig.canvas.draw()
                self.updated = False
    def getImage(self):
        with self.lock:
            img = np.fromstring(self.fig.canvas.tostring_rgb(), dtype=np.uint8,sep='')
            img  = img.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
            img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        return img

p_SLAM = fancy_matplot("SLAM")
p_AIV = fancy_matplot("AIV")
shouldClose = False
thread = Thread(target=lambda:asyncio.run(main()))
thread.start()
# asyncio.run(main())
while True:
    p_SLAM.draw()
    p_AIV.draw()
    if thread.is_alive():
        plt.pause(0.1)
    else:
        break

shouldClose = True
plt.pause(0)
print('waiting for websocket to join...')
thread.join()
print('all done :)')