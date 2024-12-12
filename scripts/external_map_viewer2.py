import asyncio
import cv2
import websockets
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread, Lock, Event

def parse_binary_data(binary_data):
        isFromSLAM = binary_data[0]
        if isFromSLAM==1:
            # rotation_matrix = np.frombuffer(binary_data[1:37], dtype=np.float32).reshape(3, 3)
            pose_data = np.frombuffer(binary_data, dtype=np.float32, count=6, offset=37)
            return isFromSLAM, pose_data
        else:
            pose_data = np.frombuffer(binary_data, dtype=np.float32, count=3, offset=1)
            return isFromSLAM, pose_data

async def main(event: Event):
    uri = "ws://192.168.1.1:9002"
    try:
        async with websockets.connect(uri, ping_interval=None) as websocket:
            print(f"Connected to WebSocket server: {uri}")

            while not event.is_set():
                binary_data = await websocket.recv()
                isFromSLAM, pose_data = parse_binary_data(binary_data)

                if isFromSLAM:
                    p_SLAM.update(pose_data[0], pose_data[1], pose_data[3], pose_data[4], doDraw=False)

                else:
                    p_AIV.update(pose_data[0], pose_data[1], doDraw=False)

    except websockets.exceptions.ConnectionClosed as e:
        print(e)
    except Exception as e:
        print(f"Error processing socket message: {e}")
    finally:
        print("WebSocket closed.")

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
        self.pos, = self.ax.plot([], [], label="Position")
        self.odom, = self.ax.plot([], [], label="Odometry")
        self.pos_datX=[]
        self.pos_datY=[]
        self.odom_datX=[0]
        self.odom_datY=[0]
        self.lock = Lock()
        if not name is None:
            self.ax.set_title(name)
        self.minX = float('inf')
        self.maxX = -float('inf')
        self.minY = float('inf')
        self.maxY = -float('inf')
        self.updated = False
        self.ax.legend()

    def update(self, pos_valX, pos_valY, odom_valX=0, odom_valY=0, doDraw=True):
        updated = False
        with self.lock:
            if pos_valX < self.minX: self.minX = pos_valX
            if pos_valY < self.minY: self.minY = pos_valY
            if pos_valX > self.maxX: self.maxX = pos_valX
            if pos_valY > self.maxY: self.maxY = pos_valY
            # print(np.sqrt((valX-self.pos_datX[-1])**2 + (valY-self.datY[-1])**2))
            if len(self.pos_datX) < 4 or len(self.pos_datY) < 4 or (not (np.sqrt((pos_valX-self.pos_datX[-1])**2 + (pos_valY-self.pos_datY[-1])**2) < 0.0001)):
                self.pos_datX.append(pos_valX)
                self.pos_datY.append(pos_valY)

                self.odom_datX.append(self.odom_datX[-1] + odom_valX)
                self.odom_datY.append(self.odom_datY[-1] + odom_valY)

                self.updated = True
                updated = True
        
        if doDraw:
            self.draw()
        
        return updated

    def draw(self):
        with self.lock:
            if self.updated:
                self.pos.set_ydata(self.pos_datY)
                self.pos.set_xdata(self.pos_datX)
                self.odom.set_ydata(self.odom_datY)
                self.odom.set_xdata(self.odom_datX)
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

if __name__ == "__main__":
    p_SLAM = fancy_matplot("SLAM")
    p_AIV = fancy_matplot("AIV")

    event = Event()
    thread = Thread(target=lambda:asyncio.run(main(event)))
    thread.start()

    # asyncio.run(main())
    try:
        while True:
            p_SLAM.draw()
            p_AIV.draw()
            if thread.is_alive():
                plt.pause(0.1)
            else:
                break
    except:
        event.set()
    finally:
        plt.close()
        print('waiting for websocket to join...')
        thread.join()
        print('all done :)')