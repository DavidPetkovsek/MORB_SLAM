import pyrealsense2 as rs
import numpy as np
import ctypes
import struct
import time
from statistics import stdev, covariance
import math
import matplotlib.pyplot as plt

def init_accel():
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.accel, format=rs.format.motion_xyz32f, framerate=250)
    pipeline.start(config)
    return pipeline

def init_gyro():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.gyro, format=rs.format.motion_xyz32f, framerate=400)
    pipeline.start(config)
    return pipeline

def get_magnitude(x, y, z):
    return math.sqrt(x**2+y**2+z**2)

accel_pipeline = init_accel()
gyro_pipeline = init_gyro()

a_array = []
a_timestamps = []
a_stdevs = []
w_array = []
w_timestamps = []
w_stdevs = []


N = 400000
s = 1000

w_t0 = 0
a_t0 = 0

while len(a_array) == 0 and len(w_array) == 0:
    accel_frame = accel_pipeline.poll_for_frames()
    gyro_frame = gyro_pipeline.poll_for_frames()

    if gyro_frame and len(w_array) == 0:
        w = gyro_frame[0].as_motion_frame().get_motion_data()
        w_array.append(get_magnitude(w.x, w.y, w.z)/3)
        w_timestamps.append(0)
        w_t0 = gyro_frame.timestamp/1000
    if accel_frame and len(a_array) == 0:
        a = accel_frame[0].as_motion_frame().get_motion_data()
        a_array.append(get_magnitude(a.x, a.y, a.z)/3)
        a_timestamps.append(0)
        a_t0 = accel_frame.timestamp/1000

while len(w_array) < N:
    accel_frame = accel_pipeline.poll_for_frames()
    gyro_frame = gyro_pipeline.poll_for_frames()

    if gyro_frame:
        w = gyro_frame[0].as_motion_frame().get_motion_data()
        w_array.append(get_magnitude(w.x, w.y, w.z)/3)
        w_timestamps.append(gyro_frame.timestamp/1000-w_t0)
    if accel_frame:
        a = accel_frame[0].as_motion_frame().get_motion_data()
        a_array.append(get_magnitude(a.x, a.y, a.z)/3)
        a_timestamps.append(accel_frame.timestamp/1000-a_t0)

accel_pipeline.stop()
gyro_pipeline.stop()

print(len(a_array), len(w_array))

a_stdev = stdev(a_array)
w_stdev = stdev(w_array)

print("IMU.NoiseAcc: " + str(a_stdev))

print("IMU.NoiseGyro: " + str(w_stdev))

a_use_timestamps = []
a_size = int((len(a_array)+1)/s)
for i in range(1, a_size):
    a_stdevs.append(stdev(a_array[:i*s]))
    a_use_timestamps.append(a_timestamps[i*s]-a_timestamps[0])

a_covar = covariance(a_timestamps, a_array)
print("IMU.AccWalk: " + str(a_covar))

w_covar = covariance(w_timestamps, w_array)
print("IMU.GyroWalk: " + str(w_covar))


figure, axis = plt.subplots(2, 1)

axis[0].plot(a_use_timestamps, a_stdevs)
axis[0].set_title("Accel STDEVs")

axis[1].plot(a_timestamps, a_array)
axis[1].set_title("Accel Magnitudes")

plt.show()