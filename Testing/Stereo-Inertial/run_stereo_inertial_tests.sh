echo "Running tests for Stereo-Inertial SLAM!"
echo -e "--------------------------------\n"

# EuRoC dataset
# Usage: ../../bin/test_stereo_inertial_euroc <path_to_vocabulary> <path_to_camera_settings> <path_to_euroc_sequence_folder> <path_to_output_csvfile>
pathDatasetEuroc="/home/david/vSLAM_datasets/EuRoC" # Change this!! Path to the folder containing your EuRoC dataset sequences, specific sequences from the dataset can be downloaded here: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
pathResultsEuroc="/home/david/vSLAM_results/EuRoC"
echo "Running EuRoC tests!"

echo "Testing on MH_02_easy..."
../../bin/test_stereo_inertial_euroc ../../Vocabulary/ORBvoc.txt ./EuRoC/EuRoC.yaml "$pathDatasetEuroc"/MH_02_easy $pathResultsEuroc/MH_02_easy/results.csv

echo "Testing on V2_02_medium..."
../../bin/test_stereo_inertial_euroc ../../Vocabulary/ORBvoc.txt ./EuRoC/EuRoC.yaml "$pathDatasetEuroc"/V2_02_medium $pathResultsEuroc/V2_02_medium/results.csv

# Manually recorded dataset
# Usage: ../../bin/test_stereo_inertial_recordings <path_to_vocabulary> <path_to_camera_settings> <path_to_recorded_sequence> <path_to_output_csvfile>
pathDatasetRecordings="/home/david/vSLAM_datasets/Recordings/Stereo-Inertial/RealSense_D435i" # Change this!! Path to the folder storing your recorded sequences
pathResultsRecordings="/home/david/vSLAM_results/Recordings/Stereo-Inertial/RealSense_D435i"
echo "Testing on data recorded by RealSense_D435i"

echo "Testing on floor2_main_hallway..."  
../../bin/test_stereo_inertial_recordings ../../Vocabulary/ORBvoc.txt ./Recordings/RealSense_D435i.yaml "$pathDatasetRecordings"/floor2_main_hallway" $pathResultsRecordings"/floor2_main_hallway/results.csv