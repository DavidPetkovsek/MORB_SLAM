import asyncio
import websockets
import numpy as np
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
    uri = "ws://172.17.124.222:9002"

    figSLAM = plt.figure()
    figAIV = plt.figure()
    axSLAM = figSLAM.add_subplot(111, projection=Axes3D.name)
    axAIV = figAIV.add_subplot(111, projection=Axes3D.name)
    axSLAM.view_init(90, -90)
    axAIV.view_init(90, -90)
    positionSLAM = np.zeros((0, 3))
    positionAIV = np.zeros((0, 3))

    scSLAM = axSLAM.scatter(positionSLAM[:, 0], positionSLAM[:, 1], positionSLAM[:, 2], s=5, c='b')
    scAIV = axAIV.scatter(positionAIV[:, 0], positionAIV[:, 1], positionAIV[:, 2], s=5, c='r')

    axSLAM.set_xlabel('X')
    axSLAM.set_ylabel('Y')
    axSLAM.set_zlabel('Z')
    axSLAM.set_title('SLAM')

    axAIV.set_xlabel('X')
    axAIV.set_ylabel('Y')
    axAIV.set_zlabel('Z')
    axAIV.set_title('AIV')
   
    plt.ion()
    plt.show()

    mins = [-1, -1, -1]
    maxs = [1, 1, 1]

    axSLAM.set_xlim(mins[0], maxs[0])
    axSLAM.set_ylim(mins[1], maxs[1])
    axSLAM.set_zlim(mins[2], maxs[2])

    axAIV.set_xlim(mins[0], maxs[0])
    axAIV.set_ylim(mins[1], maxs[1])
    axAIV.set_zlim(mins[2], maxs[2])

    async with websockets.connect(uri, ping_interval=None) as websocket:
        n = 0
        while True:
            
            binary_data = await websocket.recv()
            # isFromSLAM, rotation_matrix, translation_vector, state, message, isKF = parse_binary_data(binary_data)
            isFromSLAM, translation_vector = parse_binary_data(binary_data)
            n += 1
            # Print received data
            # print("Rotation Matrix:")
            # print(rotation_matrix)
            # print("\nTranslation Vector:")
            # print(state)
            # print("\nMessage:", message)
            # print("isKF:", isKF)
            
            if isFromSLAM:
                positionSLAM = np.vstack((positionSLAM, translation_vector))
                scSLAM._offsets3d = (positionSLAM[:, 0], positionSLAM[:, 1], positionSLAM[:, 2])
            else:
                positionAIV = np.vstack((positionAIV, translation_vector))
                scAIV._offsets3d = (positionAIV[:, 0], positionAIV[:, 1], positionAIV[:, 2])
            
            if translation_vector[0] < mins[0] or translation_vector[1] < mins[1]:
                mins[0] *= 2
                mins[1] *= 2
                axSLAM.set_xlim(mins[0], maxs[0])
                axSLAM.set_ylim(mins[1], maxs[1])
                axAIV.set_xlim(mins[0], maxs[0])
                axAIV.set_ylim(mins[1], maxs[1])
                
            elif translation_vector[0] > maxs[0] or translation_vector[1] > maxs[1]:
                maxs[0] *= 2
                maxs[1] *= 2
                axSLAM.set_xlim(mins[0], maxs[0])
                axSLAM.set_ylim(mins[1], maxs[1])
                axAIV.set_xlim(mins[0], maxs[0])
                axAIV.set_ylim(mins[1], maxs[1])

            if translation_vector[2] < mins[2]:
                mins[2] *= 2
                axSLAM.set_zlim(mins[2], maxs[2])
                axAIV.set_zlim(mins[2], maxs[2])
            elif translation_vector[2] > maxs[2]:
                maxs[2] *= 2
                axSLAM.set_zlim(mins[2], maxs[2])
                axAIV.set_zlim(mins[2], maxs[2])

            if n == 8:
                plt.pause(0.01)
                plt.draw()
                n=0

asyncio.run(main())