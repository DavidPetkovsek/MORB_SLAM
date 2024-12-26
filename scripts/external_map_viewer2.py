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
                    p_SLAM.update(pose_data[0], pose_data[1], pose_data[2], pose_data[3], pose_data[4], pose_data[5], doDraw=False)

                # else:
                #     p_AIV.update(pose_data[0], pose_data[1], pose_data[2], doDraw=False)

    except websockets.exceptions.ConnectionClosed as e:
        print(e)
    except Exception as e:
        print(f"Error processing socket message: {e}")
    finally:
        print("WebSocket closed.")


class fancy_matplot:
    def __init__(self, name=None):
        self.fig = plt.figure(figsize=(8,8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.pos, = self.ax.plot([], [], [], label="Position")
        self.odom, = self.ax.plot([], [], [], label="Odometry")
        self.pos_datX=[]
        self.pos_datY=[]
        self.pos_datZ=[]
        self.odom_datX=[]
        self.odom_datY=[]
        self.odom_datZ=[]
        self.lock = Lock()
        if not name is None:
            self.ax.set_title(name)
        axis_bound = 1.0
        self.minX = -axis_bound
        self.maxX = axis_bound
        self.minY = -axis_bound
        self.maxY = axis_bound
        self.minZ = -axis_bound
        self.maxZ = axis_bound
        self.updated = False
        self.ax.legend()
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

    def update(self, pos_valX, pos_valY, pos_valZ, odom_valX=0, odom_valY=0, odom_valZ=0, doDraw=True):
        updated = False
        with self.lock:
            # if len(self.pos_datX) < 4 or len(self.pos_datY) < 4 or (not (np.sqrt((pos_valX-self.pos_datX[-1])**2 + (pos_valY-self.pos_datY[-1])**2) < 0.0001)):
            self.pos_datX.append(pos_valX)
            self.pos_datY.append(pos_valY)
            self.pos_datZ.append(pos_valZ)

            self.odom_datX.append(odom_valX)
            self.odom_datY.append(odom_valY)
            self.odom_datZ.append(odom_valZ)

            self.minX = min(self.minX, pos_valX, odom_valX)
            self.minY = min(self.minY, pos_valY, odom_valY)
            self.minZ = min(self.minZ, pos_valZ, odom_valZ)

            self.maxX = max(self.maxX, pos_valX, odom_valX)
            self.maxY = max(self.maxY, pos_valY, odom_valY)
            self.maxZ = max(self.maxZ, pos_valZ, odom_valZ)

            self.updated = True
            updated = True
        
        if doDraw:
            self.draw()
        
        return updated

    def draw(self):
        with self.lock:
            if self.updated:
                self.pos.set_data_3d(self.pos_datX, self.pos_datY, self.pos_datZ)
                self.odom.set_data_3d(self.odom_datX, self.odom_datY, self.odom_datZ)
                self.ax.set_xlim(self.minX, self.maxX)
                self.ax.set_ylim(self.minY, self.maxY)
                self.ax.set_zlim(self.minZ, self.maxZ)
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
    # p_AIV = fancy_matplot("AIV")

    event = Event()
    thread = Thread(target=lambda:asyncio.run(main(event)))
    thread.start()

    # asyncio.run(main())
    try:
        while True:
            p_SLAM.draw()
            # p_AIV.draw()
            if thread.is_alive():
                plt.pause(0.1)
            else:
                break
    except Exception as e:
        print(e)
        event.set()
    finally:
        plt.close()
        print('waiting for websocket to join...')
        thread.join()
        print('all done :)')