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
    uri = "ws://192.168.1.1:9002"

    figSLAM = plt.figure()
    figAIV = plt.figure()
    # axSLAM = figSLAM.add_subplot(111, projection=Axes3D.name)
    # axAIV = figAIV.add_subplot(111, projection=Axes3D.name)
    axSLAM = figSLAM.add_subplot(111)
    axAIV = figAIV.add_subplot(111)
    # axSLAM.view_init(90, -90)
    # axAIV.view_init(90, -90)
    positionSLAM = np.zeros((0, 2))
    positionAIV = np.zeros((0, 2))

    # scSLAM = axSLAM.scatter(positionSLAM[:, 0], positionSLAM[:, 1], positionSLAM[:, 2], s=5, c='b')
    # scAIV = axAIV.scatter(positionAIV[:, 0], positionAIV[:, 1], positionAIV[:, 2], s=5, c='r')

    # scSLAM = axSLAM.scatter(positionSLAM[:, 0], positionSLAM[:, 1], s=5, c='b')
    # scAIV = axAIV.scatter(positionAIV[:, 0], positionAIV[:, 1], s=5, c='r')

    axSLAM.set_xlabel('X')
    axSLAM.set_ylabel('Y')
    # axSLAM.set_zlabel('Z')
    axSLAM.set_title('SLAM')

    axAIV.set_xlabel('X')
    axAIV.set_ylabel('Y')
    # axAIV.set_zlabel('Z')
    axAIV.set_title('AIV')
   
    plt.ion()
    plt.show()
    # mins = [-1, -1, -1]
    # maxs = [1, 1, 1]

    # axSLAM.set_xlim(mins[0], maxs[0])
    # axSLAM.set_ylim(mins[1], maxs[1])
    # axSLAM.set_zlim(mins[2], maxs[2])

    # axAIV.set_xlim(mins[0], maxs[0])
    # axAIV.set_ylim(mins[1], maxs[1])
    # axAIV.set_zlim(mins[2], maxs[2])

    try:
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
                    # print("A")
                    positionSLAM = np.vstack((positionSLAM, translation_vector[:2]))
                    
                    # scSLAM.set_offsets(positionSLAM[:, 0], positionSLAM[:, 1])
                    # print("c")
                else:
                    # print("d")
                    positionAIV = np.vstack((positionAIV, translation_vector[:2]))
                    
                    # scAIV._offsets3d = (positionAIV[:, 0], positionAIV[:, 1], positionAIV[:, 2])
                    # scAIV.set_offsets(np.column_stack((positionAIV[:, 0], positionAIV[:, 1])))
                
                # if translation_vector[0] < mins[0] or translation_vector[1] < mins[1]:
                #     mins[0] *= 2
                #     mins[1] *= 2
                #     axSLAM.set_xlim(m`ins[0], maxs[0])
                #     axSLAM.set_ylim(mins[1], maxs[1])
                #     axAIV.set_xlim(mins[0], maxs[0])
                #     axAIV.set_ylim(mins[1], maxs[1])
                    
                # elif translation_vector[0] > maxs[0] or translation_vector[1] > maxs[1]:
                #     maxs[0] *= 2
                #     maxs[1] *= 2
                #     axSLAM.set_xlim(mins[0], maxs[0])
                #     axSLAM.set_ylim(mins[1], maxs[1])
                #     axAIV.set_xlim(mins[0], maxs[0])
                #     axAIV.set_ylim(mins[1], maxs[1])

                # if translation_vector[2] < mins[2]:
                #     mins[2] *= 2
                #     axSLAM.set_zlim(mins[2], maxs[2])
                #     axAIV.set_zlim(mins[2], maxs[2])
                # elif translation_vector[2] > maxs[2]:
                #     maxs[2] *= 2
                #     axSLAM.set_zlim(mins[2], maxs[2])
                #     axAIV.set_zlim(mins[2], maxs[2])

                if n == 10:
                    n=0
                    axAIV.plot(positionAIV[:, 0], positionAIV[:, 1], 'o', markersize=2)
                    axSLAM.plot(positionSLAM[:, 0], positionSLAM[:, 1], 'o', markersize=2)
                    plt.pause(0.1)
                    # plt.draw()
    except:
        axSLAM.plot(positionSLAM[:, 0], positionSLAM[:, 1], 'o', markersize=3)
        axAIV.plot(positionAIV[:, 0], positionAIV[:, 1], 'o', markersize=3)
        plt.pause(0.01)
        plt.draw()
        while True:
            if plt.waitforbuttonpress():
                event = plt.gcf().canvas.manager.key_press_handler
                if event.key == 'q':
                    plt.close()
                    break


asyncio.run(main())