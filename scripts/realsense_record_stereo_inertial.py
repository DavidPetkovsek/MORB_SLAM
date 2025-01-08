'''
Run this script with your IntelRealsense D435i camera plugged in to collect time stamped IMU and stereo data.
Arguments: path_to_directory, name_of_dataset
For example: `python realsense_record_stereo_inertial /home/david/vSLAM_datasets/Recordings/Stereo-Inertial/RealSense_D435i/ main_hallway`
'''

import sys
import os
import pyrealsense2 as rs
import csv
import numpy as np
import cv2
import shutil
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output_path", help="Specify the directory where the recorded camera and IMU data will be written to")
    parser.add_argument("--fps", type=int, default=15)
    
    args = parser.parse_args()
    data_dirs = [
        os.path.join(args.output_path, "cam0"),
        os.path.join(args.output_path, "cam1"),
        os.path.join(args.output_path, "IMU"),
    ]

    if os.path.exists(args.output_path): 
        while True:
            user_input = input(f"The folder {args.output_path} already exists, would you like to overwrite it with your recorded data? (y/N)")
            if user_input == 'y':
                print("The newly recorded data will overwrite the existing folder.")
                break
            elif user_input == 'N':
                print("Cancelling the recording process. the program will now exit.")
                sys.exit(0)
            else:
                print("Invalid input, try again.")

        for dir in data_dirs:
            if os.path.exists(dir):
                print(f"Removing {dir}...")
                shutil.rmtree(dir)

            print(f"Creating {dir}...")
            os.makedirs(dir)
    else:
        for dir in data_dirs:
            print(f"Creating {dir}...")
            os.makedirs(dir)

    cam_pipeline = rs.pipeline()
    cam_config = rs.config()
    cam_config.enable_stream(rs.stream.infrared, stream_index=1, width=640, height=480, format=rs.format.y8, framerate=args.fps) #left cam
    cam_config.enable_stream(rs.stream.infrared, stream_index=2, width=640, height=480, format=rs.format.y8, framerate=args.fps) #right cam

    accel_pipeline = rs.pipeline()
    accel_config = rs.config()
    accel_config.enable_stream(rs.stream.accel, format=rs.format.motion_xyz32f, framerate=250)

    gyro_pipeline = rs.pipeline()
    gyro_config = rs.config()
    gyro_config.enable_stream(rs.stream.gyro, format=rs.format.motion_xyz32f, framerate=200)

    cam_pipeline.start(cam_config)
    accel_pipeline.start(accel_config)
    gyro_pipeline.start(gyro_config)

    cam_pipeline_profile = cam_pipeline.get_active_profile()
    depth_sensor = cam_pipeline_profile.get_device().query_sensors()[0]
    laser_range = depth_sensor.get_option_range(rs.option.laser_power)
    depth_sensor.set_option(rs.option.laser_power, laser_range.min)

    cam_frame_count = 0 # keep track of how many frames we've read

    try:
        with open(os.path.join(args.output_path, "IMU", "acc.csv"), 'w', newline='') as accel_csvfile, open(os.path.join(args.output_path, "IMU", "gyro.csv"), 'w', newline='') as gyro_csvfile, open(os.path.join(args.output_path, "cam0", "times.csv"), 'w', newline='') as cam_csvfile:
            accel_csv_writer = csv.writer(accel_csvfile, delimiter=',')
            accel_csv_writer.writerow(["#timestamp [ms]", "a_x [m s^-2]", "a_y [m s^-2]", "a_z [m s^-2]"])

            gyro_csv_writer = csv.writer(gyro_csvfile, delimiter=',')
            gyro_csv_writer.writerow(["#timestamp [ms]", "w_x [rad s^-1]", "w_y [rad s^-1]", "w_z [rad s^-1]"])

            cam_csv_writer = csv.writer(cam_csvfile, delimiter=',')
            cam_csv_writer.writerow(["#timestamp [ms]"])

            print("Recording started...")

            cam_pipeline.wait_for_frames(10000)
            gyro_pipeline.wait_for_frames(10000)
            accel_pipeline.wait_for_frames(10000)

            while True:
                accel_frames = accel_pipeline.poll_for_frames()
                gyro_frames = accel_pipeline.poll_for_frames()
                cam_frames = cam_pipeline.poll_for_frames()

                if accel_frames:
                    accel_timestamp = accel_frames.get_frame_metadata(rs.frame_metadata_value.backend_timestamp) # backend timestamp is used because the frame timestamp is not from epoch
                    accel_data = accel_frames[0].as_motion_frame().get_motion_data()
                    accel_csv_writer.writerow([
                        accel_timestamp,
                        accel_data.x,
                        accel_data.y,
                        accel_data.z,
                    ])
                
                if gyro_frames:
                    gyro_timestamp = gyro_frames.get_frame_metadata(rs.frame_metadata_value.backend_timestamp) # backend timestamp is used because the frame timestamp is not from epoch
                    gyro_data = gyro_frames[0].as_motion_frame().get_motion_data()
                    gyro_csv_writer.writerow([
                        gyro_timestamp,
                        gyro_data.x,
                        gyro_data.y,
                        gyro_data.z,
                    ])
                
                if cam_frames:
                    left_cam_frame = np.asarray(cam_frames[0].get_data())
                    right_cam_frame = np.asarray(cam_frames[1].get_data())

                    cam_timestamp = cam_frames.get_timestamp()

                    cv2.imshow("Left Camera", left_cam_frame)
                    cv2.imwrite(os.path.join(args.output_path, 'cam0', str(cam_timestamp) + '.png'), left_cam_frame)
                    cv2.imwrite(os.path.join(args.output_path, 'cam1', str(cam_timestamp) + '.png'), right_cam_frame)
                    cam_csv_writer.writerow([cam_timestamp])

                    cam_frame_count += 1

                    cv2.waitKey(1) # 1 millisecond, just to display the image

    except KeyboardInterrupt:
        print("Caught KeyboardInterrupt!")

    finally:
        print("Exiting...")
        cam_pipeline.stop()
        accel_pipeline.stop()
        gyro_pipeline.stop()

        cam_csvfile.close()
        accel_csvfile.close()
        gyro_csvfile.close()
        print(f"Finished recording {cam_frame_count} frames!")
        


if __name__ == "__main__":
    main()

