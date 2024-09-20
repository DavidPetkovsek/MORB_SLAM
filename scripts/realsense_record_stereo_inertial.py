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


def main():

    if len(sys.argv) != 3:
        sys.exit("Usage: python realsense_record_stereo_inertial <path_to_directory> <name_of_dataset>")
    
    output_path = os.path.join(sys.argv[1], sys.argv[2])
    cam_data_dirs = (os.path.join(output_path, "cam0"), os.path.join(output_path, "cam1"))

    if os.path.exists(output_path): 
        while True:
            user_input = input(f"The folder {output_path} already exists, would you like to overwrite it with your recorded data? (y/N)")
            if user_input == 'y':
                print("The newly recorded data will overwrite the existing folder.")
                break
            elif user_input == 'N':
                print("Cancelling the recording process. the program will now exit.")
                sys.exit(0)
            else:
                print("Invalid input, try again.")

        for dir in cam_data_dirs:
            if not os.path.exists(dir):
                print(f"Directory {dir} does not exist, creating one...")
                os.makedirs(dir)
            else:
                print(f"Directory {dir} exists, removing it's contents...")
                for item in os.listdir(dir):
                    item_path = os.path.join(dir, item)
                    if os.path.isfile(item_path):
                        os.remove(item_path) 
                    elif os.path.isdir(item_path):
                        shutil.rmtree(item_path)
    else:
        print(f"Creating {cam_data_dirs[0]}...")
        os.makedirs(cam_data_dirs[0])
        print(f"Creating {cam_data_dirs[1]}...")
        os.makedirs(cam_data_dirs[1])

    # A single pipeline will return synchronized streams. The camera will have a seperate pipeline since we want the IMU stream to come in more frequently
    cam_pipeline = rs.pipeline()
    cam_config = rs.config()
    cam_config.enable_stream(rs.stream.infrared, stream_index=1, width=640, height=480, format=rs.format.y8, framerate=15) #left cam
    cam_config.enable_stream(rs.stream.infrared, stream_index=2, width=640, height=480, format=rs.format.y8, framerate=15) #right cam

    imu_pipeline = rs.pipeline()
    imu_config = rs.config()
    imu_config.enable_stream(rs.stream.accel, format=rs.format.motion_xyz32f, framerate=250)
    imu_config.enable_stream(rs.stream.gyro, format=rs.format.motion_xyz32f, framerate=200)

    cam_pipeline.start(cam_config)
    imu_pipeline.start(imu_config)

    cam_pipeline_profile = cam_pipeline.get_active_profile()
    depth_sensor = cam_pipeline_profile.get_device().query_sensors()[0]
    laser_range = depth_sensor.get_option_range(rs.option.laser_power)
    depth_sensor.set_option(rs.option.laser_power, laser_range.min)

    cam_frame_count = 0 # keep track of how many frames we've read
    prev_imu_timestamp = None

    try:
        with open(os.path.join(output_path, "imu_data.csv"), 'w', newline='') as imu_csvfile, open(os.path.join(output_path, "cam_data.csv"), 'w', newline='') as cam_csvfile:
            imu_csv_writer = csv.writer(imu_csvfile, delimiter=',')
            imu_csv_writer.writerow(["#timestamp [ms]", "w_x [rad s^-1]", "w_y [rad s^-1]", "w_z [rad s^-1]", "a_x [m s^-2]", "a_y [m s^-2]", "a_z [m s^-2]"])

            cam_csv_writer = csv.writer(cam_csvfile, delimiter=',')
            cam_csv_writer.writerow(["#timestamp [ms]"])

            print("Recording started...")

            imu_pipeline.wait_for_frames(10000)
            cam_pipeline.wait_for_frames(10000)
            
            while cam_frame_count < 900: # max 60 seconds
                imu_frames = imu_pipeline.poll_for_frames()
                cam_frames = cam_pipeline.poll_for_frames()

                if imu_frames:
                    accel_frame = imu_frames[0]
                    gyro_frame = imu_frames[1]

                    imu_timestamp = imu_frames.get_frame_metadata(rs.frame_metadata_value.backend_timestamp) # backend timestamp is used because the frame timestamp is not from epoch

                    # This if statement is used because calling poll_for_frames on the imu_stream results in consecutive frames with the same timestamp
                    if imu_timestamp != prev_imu_timestamp:
                        accel_data = accel_frame.as_motion_frame().get_motion_data()
                        gyro_data = gyro_frame.as_motion_frame().get_motion_data()

                        imu_csv_writer.writerow([
                            imu_timestamp,
                            gyro_data.x,
                            gyro_data.y,
                            gyro_data.z,
                            accel_data.x,
                            accel_data.y,
                            accel_data.z,
                        ])

                        prev_imu_timestamp = imu_timestamp 
                
                if cam_frames:
                    left_cam_frame = np.asarray(cam_frames[0].get_data())
                    right_cam_frame = np.asarray(cam_frames[1].get_data())

                    cam_timestamp = cam_frames.get_timestamp()

                    cv2.imwrite(os.path.join(output_path, 'cam0', str(cam_timestamp) + '.png'), left_cam_frame)
                    cv2.imwrite(os.path.join(output_path, 'cam1', str(cam_timestamp) + '.png'), right_cam_frame)
                    cam_csv_writer.writerow([cam_timestamp])

                    cam_frame_count += 1

    except KeyboardInterrupt:
        print("Caught KeyboardInterrupt!")

    finally:
        print("Exiting...")
        cam_pipeline.stop()
        imu_pipeline.stop()
        imu_csvfile.close()
        cam_csvfile.close()
        print("Done :)")
        


if __name__ == "__main__":
    main()

