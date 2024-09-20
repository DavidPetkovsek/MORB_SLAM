#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include <ctime>
#include <sstream>
#include <filesystem>

#include <opencv2/core/core.hpp>

#include <MORB_SLAM/System.h>
#include <MORB_SLAM/Viewer.h>
#include <MORB_SLAM/ExternalMapViewer.h>
#include "MORB_SLAM/ExternalIMUProcessor.h"

#include "MORB_SLAM/ImuTypes.h"


bool load_images(const std::filesystem::path &path_left_images, const std::filesystem::path &path_right_images, const std::filesystem::path &path_cam_csv,
                std::vector<std::string> &v_str_image_left, std::vector<std::string> &v_str_image_right, std::vector<double> &v_timestamp_cam);

bool load_imu(const std::filesystem::path &path_imu, std::vector<double> &v_timestamp_imu, std::vector<cv::Point3f> &v_acc, std::vector<cv::Point3f> &v_gyro);

void write_imgs_to_video(const std::filesystem::path &output_vid_path, const std::vector<std::string> &v_img_paths, double frame_rate);

void write_pose_to_results(std::ofstream &results_file, Sophus::SE3f &pose, double timestamp);

int main(int argc, char **argv)
{
    if(argc < 4) {
        std::cerr << "\nRequired arguments: <path_to_vocabulary> <path_to_camera_settings> <path_to_recording_sequence_folder> <results_file_path>" << std::endl;
        return 1;
    }

    std::vector<std::string> v_str_image_left; // vector containing path to each left frame in the sequence
    std::vector<std::string> v_str_image_right; // vector containing path to each right frame in the sequence
    std::vector<double> v_timestamp_cam_s; // vector containing timestamps (in seconds) of each frame in the sequence (left and right frames are synchronized)

    std::vector<cv::Point3f> v_acc, v_gyro;
    std::vector<double> v_timestamp_imu_s; // IMU timestamps at each measurement (in seconds)
    
    int num_images;
    int num_imu;
    int first_imu_idx;
    int first_cam_idx;

    std::filesystem::path path_to_seq(argv[3]);
    std::filesystem::path path_cam0_images = path_to_seq / "cam0";
    std::filesystem::path path_cam1_images = path_to_seq / "cam1";
    std::filesystem::path path_cam_csv = path_to_seq / "cam_data.csv";
    std::filesystem::path path_imu_csv = path_to_seq / "imu_data.csv";

    bool b_results_file;
    std::filesystem::path path_results_csv;
    std::ofstream results_file;
    if(argc == 5) {
        path_results_csv = argv[4];
        std::filesystem::create_directories(path_results_csv.parent_path());
        results_file.open(path_results_csv); // csv file containing timestamped poses calculated by MORB_SLAM
        if(!results_file.is_open()) {
            std::cerr << "ERROR: Unable to open the results file: " << path_results_csv.string() << "... NOT writing the results." << std::endl;
            b_results_file = false;
        } else {
            std::cout << "Results file specified: " << path_results_csv.string() << std::endl;
            b_results_file = true;
            results_file << "#timestamp, p_x, p_y, p_z, q_x, q_y, q_z, q_w" << std::endl;
        }
    }

    std::cout << "Loading images for " << path_to_seq << "..." << std::endl;
    if(!load_images(path_cam0_images, path_cam1_images, path_cam_csv, v_str_image_left, v_str_image_right, v_timestamp_cam_s)) {
        std::cerr << "ERROR: Failed to load images for " << path_to_seq << std::endl;
        return 1;
    }

    std::cout << "Loading IMU FOR " << path_to_seq << "..." << std::endl;
    if(!load_imu(path_imu_csv, v_timestamp_imu_s, v_acc, v_gyro)) {
        std::cerr << "ERROR: Failed to load IMU for " << path_to_seq << std::endl;
        return 1;
    }

    num_images = v_str_image_left.size();
    num_imu = v_timestamp_imu_s.size();

    if(num_images<=0 || num_imu<=0){
        std::cerr << "ERROR: the image data or imu data does not exist! Check your data. " << path_to_seq << std::endl;
        return 1;
    } else {
        std::cout << "There are " << num_images << " stereo images and " << num_imu << " rows of IMU data." << std::endl;
    }

    // Check if camera settings can be read
    cv::FileStorage camera_settings(argv[2], cv::FileStorage::READ);
    if(!camera_settings.isOpened()) {
        std::cerr << "ERROR: Wrong path to camera settings: " << argv[2] << std::endl;
        return 1;
    }

    // Don't consider camera frames that started before the IMU
    while(v_timestamp_cam_s[first_cam_idx] < v_timestamp_imu_s[first_imu_idx]) {
        ++first_cam_idx;
    }

    // Start at the latest IMU measurement that was captured before (or during) the first camera frame
    while(v_timestamp_imu_s[first_imu_idx]<=v_timestamp_cam_s[first_cam_idx])
        ++first_imu_idx;
    --first_imu_idx;

    std::cout << "The first imu measurement to be considered is at index " << first_imu_idx << std::endl;

    // Write the image sequences to a video, if a video doesn't exist
    std::filesystem::path output_vid_path_left = path_to_seq / "stereo_left.avi";
    std::filesystem::path output_vid_path_right = path_to_seq / "stereo_right.avi";
    double frame_rate;
    camera_settings["Camera.fps"] >> frame_rate;

    if(!std::filesystem::exists(output_vid_path_left)) {
        std::cout << "Creating a video using the left image sequence: " << output_vid_path_left << std::endl;
        write_imgs_to_video(output_vid_path_left, v_str_image_left, frame_rate);
    }

    if (!std::filesystem::exists(output_vid_path_right)) {
        std::cout << "Creating a video using the right image sequence: " << output_vid_path_right << std::endl;
        write_imgs_to_video(output_vid_path_right, v_str_image_right, frame_rate);
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    auto SLAM = std::make_shared<MORB_SLAM::System>(argv[1],argv[2], MORB_SLAM::CameraType::IMU_STEREO);
    auto viewer = std::make_shared<MORB_SLAM::Viewer>(SLAM);

    std::vector<float> v_duration_track_s; // keeping track of how long each frame took to process
    v_duration_track_s.resize(num_images);

    cv::Mat im_left, im_right;

    for(int ni = first_cam_idx; ni < num_images; ++ni) {
        im_left = cv::imread(v_str_image_left[ni], cv::IMREAD_UNCHANGED);
        im_right = cv::imread(v_str_image_right[ni], cv::IMREAD_UNCHANGED);

        if(im_left.empty()) {
            std::cerr << "ERROR: Failed to load image at: " << v_str_image_left[ni] << std::endl;
            return 1;
        }

        if(im_right.empty()) {
            std::cerr << "ERROR: Failed to load image at: " << v_str_image_right[ni] << std::endl;
            return 1;
        }
        
        double t_frame = v_timestamp_cam_s[ni];
        double t_frame_prev = ni > 0 ? v_timestamp_cam_s[ni-1] : 0.0;

        std::vector<Sophus::Vector6f> v_local_imu_meas;
        std::vector<double> v_local_timestamp_imu_s;

        while(v_timestamp_imu_s[first_imu_idx] <= v_timestamp_cam_s[ni]) {
            Sophus::Vector6f imu_meas;
            imu_meas << v_acc[first_imu_idx].x, v_acc[first_imu_idx].y, v_acc[first_imu_idx].z, v_gyro[first_imu_idx].x, v_gyro[first_imu_idx].y, v_gyro[first_imu_idx].z;
            v_local_imu_meas.push_back(imu_meas);
            v_local_timestamp_imu_s.push_back(v_timestamp_imu_s[first_imu_idx]);
            ++first_imu_idx;
        }

        // Pass the images to the SLAM system
        std::pair<double, std::vector<MORB_SLAM::IMU::Point>> slam_data = MORB_SLAM::IMUProcessor::ProcessIMU(v_local_imu_meas, v_local_timestamp_imu_s, t_frame_prev, t_frame);

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        MORB_SLAM::StereoPacket sophus_pose = SLAM->TrackStereo(im_left, im_right, slam_data.first, slam_data.second);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        viewer->update(sophus_pose);

        if (b_results_file && sophus_pose.pose.has_value()) { // write pose to results file if argument is provided
            write_pose_to_results(results_file, *sophus_pose.pose, t_frame);
        }

        double duration_s = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count(); // in seconds
        v_duration_track_s[ni] = duration_s;

        // Wait to load the next frame
        if(ni < num_images-1) {
            double T = v_timestamp_cam_s[ni+1]-t_frame; // time between current frame and next frame, in seconds
            usleep((T - duration_s) * 1e6); // usleep uses microseconds
        }
    }

    camera_settings.release();

    if(b_results_file) {
        results_file.close();
    }

    std::cout << "Stopping Viewer" << std::endl;
    viewer.reset();
    std::cout << "Stopping SLAM" << std::endl;
    SLAM.reset();
    std::cout << "Done :)" << std::endl;

    return 0;
}

// Loads the names of the left and right images from the sequence into std::vector
bool load_images(const std::filesystem::path &path_left_images, const std::filesystem::path &path_right_images, const std::filesystem::path &path_cam_csv,
                std::vector<std::string> &v_str_image_left, std::vector<std::string> &v_str_image_right, std::vector<double> &v_timestamp_cam)
{
    std::ifstream csvfile;
    csvfile.open(path_cam_csv.string().c_str());

    if(!csvfile) {
        std::cerr << "ERROR: Unable to open file: " << path_cam_csv << std::endl;
        return false;
    }

    v_timestamp_cam.reserve(5000);
    v_str_image_left.reserve(5000);
    v_str_image_right.reserve(5000);

    std::string line, word;
    while(std::getline(csvfile, line, '\n'))
    {
        if (line[0] == '#')
            continue;

        if(!line.empty()) {
            if (line.back() == '\n')
                line.pop_back();

            if (line.back() == '\r')
                line.pop_back();

            std::stringstream ss(line);

            if (std::getline(ss, word, ',')) { // first column
                v_str_image_left.push_back((path_left_images / word += ".png").string()); 
                v_str_image_right.push_back((path_right_images / word += ".png").string()); 
                double timestamp_ms = std::stod(word); // timestamp is in milliseconds, convert to seconds
                v_timestamp_cam.push_back(timestamp_ms/1e3);
            }
        }
    }

    return true;
}

// Loads the IMU data from the csv file in the sequence into std::vector
bool load_imu(const std::filesystem::path &path_imu, std::vector<double> &v_timestamp_imu_s, std::vector<cv::Point3f> &v_acc, std::vector<cv::Point3f> &v_gyro)
{
    std::ifstream csvfile;
    csvfile.open(path_imu.string().c_str());

    if(!csvfile) {
        std::cerr << "ERROR: Unable to open file: " << path_imu << std::endl;
        return false;
    }

    v_timestamp_imu_s.reserve(5000);
    v_acc.reserve(5000);
    v_gyro.reserve(5000);

    std::string line;
    while(std::getline(csvfile, line))
    {
        if(line[0] == '#')
            continue;

        if(!line.empty())
        {
            std::string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = line.find(',')) != std::string::npos) {
                item = line.substr(0, pos);
                data[count++] = std::stod(item);
                line.erase(0, pos + 1);
            }
            item = line.substr(0, pos);
            data[6] = std::stod(item);

            v_timestamp_imu_s.push_back(data[0]/1e3); // in milliseconds, convert to seconds
            v_acc.push_back(cv::Point3f(data[4],data[5],data[6]));
            v_gyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }

    return true;
}

void write_imgs_to_video(const std::filesystem::path &output_vid_path, const std::vector<std::string> &v_img_paths, double frame_rate) {
    cv::VideoWriter vid_writer;
    cv::Mat first_img = cv::imread(v_img_paths[0]);
    if (first_img.empty()) {
        std::cerr << "ERROR: Could not open or find the first image. Failed to write to video." << std::endl;
        return;
    }

    int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    vid_writer.open(output_vid_path.string().c_str(), codec, frame_rate, first_img.size());
    
    if (!vid_writer.isOpened()) {
        std::cerr << "ERROR: Could not open the output video file. Failed to write to video." << std::endl;
        return;
    }

    // Loop through all images and write them to the video
    for (const auto& img_path : v_img_paths) {
        cv::Mat image = cv::imread(img_path);
        if (image.empty()) {
            std::cerr << "Could not open or find the image: " << img_path << "... skipping this image." << std::endl;
            continue;
        }

        if (image.size() != first_img.size()) {
            cv::resize(image, image, first_img.size());
        }

        vid_writer.write(image);
    }

    vid_writer.release();
    std::cout << output_vid_path << " created successfully!" << std::endl;
}

void write_pose_to_results(std::ofstream &results_file, Sophus::SE3f &pose, double timestamp) {
    Eigen::Quaternionf q = pose.unit_quaternion();
    Eigen::Vector3f twb = pose.translation();
    results_file << std::fixed << std::setprecision(9) << timestamp << ", " << twb(0) << ", " << twb(1) << ", " << twb(2) << ", " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;
}