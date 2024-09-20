## Testing Stereo-Inertial SLAM using existing datasets
Currently, this repo supports testing against the EuRoC dataset.

### Installing the datasets
To install the EuRoC dataset, visit https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets. There are multiple test sequences with varying difficulty to choose from.

It must be installed next to `MORB_SLAM` in a folder called `vSLAM_datasets/EuRoC` if you would like to use `run_stereo_inertial_tests.sh`.

### Running tests on existing datasets
To run tests on the EuroC dataset:
```bash
# Testing/Stereo-Inertial/
../../bin/test_stereo_inertial_euroc <path_to_vocabulary> <path_to_euroc_camera_settings> <path_to_euroc_sequence_folder> <path_to_output_csvfile>
```

The `<path_to_output_csvfile>` is an optional argument. If it is omitted, the output poses from MORB_SLAM will not be written to a CSV file.

Refer to `run_stereo_inertial_tests.sh` for examples of how to run tests on existing datasets. It contains a series of commands in the format described above.

## Testing Stereo-Inertial SLAM using your own collected data
In addition to testing against existing datasets, it is possible to run tests on data that was collected manually using a camera.

### Collecting your own data using a camera
Ensure the dependencies in `scripts/requirements.txt` are installed in a Python virtual environment.

`scripts/realsense_record_stereo_inertial.py` is a Python script for recording Stereo-Inertial data using a RealSense camera.

```bash
# scripts/
python realsense_record_stereo_inertial.py <path_to_dataset> <name_of_sequence>
```

A folder called `<name_of_sequence>` will be created in `<path_to_dataset>`, and will contain the following items:
- `cam0/`: folder with .png images collected by the left camera, named by timestamp
- `cam1/`: folder with .png images collected by the right camera, named by timestamp
- `cam_data.csv`: timestamps of the of the images captured 
- `imu_data.csv`: timestamped IMU data

`<path_to_dataset>` must be next to `MORB_SLAM` as `vSLAM_datasets/Recordings/Stereo-Inertial/RealSense_D435i` if you would like to use `run_stereo_inertial_tests.sh`.

### Running tests on recorded data
To run tests on recorded Stereo-Inertial data:
```bash
# Testing/Stereo-Inertial/
../../bin/test_stereo_inertial_recordings <path_to_vocabulary> <path_to_camera_settings> <path_to_recorded_sequence> <path_to_output_csvfile>
```

The `<path_to_output_csvfile>` is an optional argument. If it is omitted, the output poses from MORB_SLAM will not be written to a CSV file.

Refer to `run_stereo_inertial_tests.sh` for examples of how to run tests on manually recorded data. It contains a series of commands in the format described above.

## Evaluation
Ensure the dependencies in `scripts/requirements.txt` are installed in a Python virtual environment.

`scripts/evaluate_stereo_inertial.py` is a Python script for evaluating output trajectories from tests ran on recorded data or existing datasets. For tests with ground truth data available, the script generates useful plots and calculates Absolute Pose Error (APE) and Relative Pose Error (RPE) between the ground truth and output trajectories using [evo](https://github.com/MichaelGrupp/evo/tree/master). For tests without ground truth data, the script will simply plot the output trajectory.

To evaluate the tests ran on the EuRoC dataset:
```bash
# scripts/
python evaluate_stereo_inertial.py euroc <path_to_ground_truth_csv> <path_to_result_csv>
```

To evaluate the tests ran on recorded data (no ground truth data available):
```bash
# scripts/
python evaluate_stereo_inertial.py recording <path_to_result_csv>
```
