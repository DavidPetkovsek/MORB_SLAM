import pyrealsense2 as rs
import numpy as np
import asyncio
from websockets.server import serve
import ctypes
import struct
import time

def init_cams():
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.infrared, stream_index=1, width=848, height=480, format=rs.format.y8, framerate=30)
    config.enable_stream(rs.stream.infrared, stream_index=2, width=848, height=480, format=rs.format.y8, framerate=30)
    pipeline.start(config)
    return pipeline

def init_accel():
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.accel, format=rs.format.motion_xyz32f, framerate=250)
    pipeline.start(config)
    return pipeline

def init_gyro():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.gyro,  format=rs.format.motion_xyz32f, framerate=200)
    pipeline.start(config)
    return pipeline

def parse_data(input):
    return np.asarray([input.x, input.y, input.z], dtype=np.float32).tobytes()

async def do_polling(websocket):
    async for message in websocket:
        cams_pipeline = init_cams()
        accel_pipeline = init_accel()
        gyro_pipeline = init_gyro()

        send_accel = False
        send_gyro = False

        real_accel = None
        real_gyro = None

        try:
            # wait until both pipelines can get a reading
            accel_frame = accel_pipeline.wait_for_frames(20000)
            gyro_frame = gyro_pipeline.wait_for_frames(20000)
            cam_frame = cams_pipeline.wait_for_frames(20000)
            t0 = cam_frame.timestamp

            while True:
                accel_frame = accel_pipeline.poll_for_frames()
                gyro_frame = gyro_pipeline.poll_for_frames()
                cam_frame = cams_pipeline.poll_for_frames()

                if accel_frame:
                    real_accel = accel_frame
                    send_accel = True

                if gyro_frame:
                    real_gyro = gyro_frame
                    send_gyro = True
                
                if cam_frame:
                    timestamp = struct.pack('<d', (cam_frame.timestamp-t0)/1000)
                    left_cam = np.asarray(cam_frame[0].get_data(), dtype=np.uint8).tobytes()
                    right_cam = np.asarray(cam_frame[1].get_data(), dtype=np.uint8).tobytes()
                    cam_frame = None
                    await websocket.send(bytes([1])+timestamp+left_cam+right_cam)

                if send_accel and send_gyro:
                    timestamp = struct.pack('<d', ((real_accel.timestamp+real_gyro.timestamp)/2-t0)/1000)
                    a = parse_data(real_accel[0].as_motion_frame().get_motion_data())
                    #a = np.asarray([0,0,-9.81], dtype=np.float32).tobytes()
                    w = parse_data(real_gyro[0].as_motion_frame().get_motion_data())
                    #w = np.asarray([0,0,0], dtype=np.float32).tobytes()
                    await websocket.send(bytes([2])+timestamp+a+w)
                    send_accel = False
                    send_gyro = False

        finally:
            cams_pipeline.stop()
            accel_pipeline.stop()
            accel_pipeline.stop()


async def main():
   async with serve(do_polling, "0.0.0.0", 8765):
       await asyncio.Future()
asyncio.run(main())