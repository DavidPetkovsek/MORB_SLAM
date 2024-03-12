import asyncio
import websockets
import numpy as np
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def parse_binary_data(binary_data):
        rotation_matrix = np.frombuffer(binary_data[:36], dtype=np.float32).reshape(3, 3)
        translation_vector = np.frombuffer(binary_data[36:48], dtype=np.float32)
        state = struct.unpack('<i', binary_data[48:52])[0]
        message = struct.unpack('<i', binary_data[52:56])[0]
        isKF = struct.unpack('<?', binary_data[56:57])[0]

        return rotation_matrix, translation_vector, state, message, isKF

async def main():
    uri = "ws://192.168.1.1:9002"

    fig = plt.figure()
    ax = fig.add_subplot(111, projection=Axes3D.name)
    ax.view_init(90, -90)
    positions = np.zeros((0, 3))

    sc = ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], s=5, c='b')

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Position Vectors')
   

    plt.ion()
    plt.show()

    mins = [-1, -1, -1]
    maxs = [1, 1, 1]

    ax.set_xlim(mins[0], maxs[0])
    ax.set_ylim(mins[1], maxs[1])
    ax.set_zlim(mins[2], maxs[2])


    async with websockets.connect(uri, ping_interval=None) as websocket:
        n = 0
        while True:
            
            binary_data = await websocket.recv()
            rotation_matrix, translation_vector, state, message, isKF = parse_binary_data(binary_data)
            n += 1
            # Print received data
            # print("Rotation Matrix:")
            # print(rotation_matrix)
            # print("\nTranslation Vector:")
            # print(state)
            # print("\nMessage:", message)
            # print("isKF:", isKF)
            
            positions = np.vstack((positions, translation_vector))
            sc._offsets3d = (positions[:, 0], positions[:, 1], positions[:, 2])
            # sc.set_color(['b'] * (len(positions) - 1) + ['r'])
            
            if translation_vector[0] < mins[0] or translation_vector[1] < mins[1]:
                mins[0] *= 2
                mins[1] *= 2
                ax.set_xlim(mins[0], maxs[0])
                ax.set_ylim(mins[1], maxs[1])
            elif translation_vector[0] > maxs[0] or translation_vector[1] > maxs[1]:
                maxs[0] *= 2
                maxs[1] *= 2
                ax.set_xlim(mins[0], maxs[0])
                ax.set_ylim(mins[1], maxs[1])

            # if translation_vector[1] < mins[1]:
            #     mins[1] *= 2
            #     ax.set_ylim(mins[1], maxs[1])
            # elif translation_vector[1] > maxs[1]:
            #     maxs[1] *= 2
            #     ax.set_ylim(mins[1], maxs[1])

            if translation_vector[2] < mins[2]:
                mins[2] *= 2
                ax.set_zlim(mins[2], maxs[2])
            elif translation_vector[2] > maxs[2]:
                maxs[2] *= 2
                ax.set_zlim(mins[2], maxs[2])

            if n == 8:
                plt.pause(0.01)
                plt.draw()
                n=0

asyncio.run(main())