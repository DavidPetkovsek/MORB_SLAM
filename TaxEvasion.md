# List of Changes to MORB-SLAM from January 2024 to August 2024

1. Script to Send Camera Data from Windows to WSL Server via Websocket
    - MORB-SLAM is designed to run on Linux, and is not supported on Windows. This can be fixed by using a WSL server to run it. However the RealSense camera firmware we use is not compatible with WSL, so we now use a websocket to send the camera data from windows to WSL. We are now able to run MORB-SLAM on Windows devices.

2. Fix Possible Divide-by-Zero When Processing IMU Measurements
    - When processing data from the IMU, ORB-SLAM3 assumes that no IMU measurement will have the same timestamp as an image. In this case the measurement would have dt=0, which leads to a division by zero. Added a check to ensure this wouldn't happen.

3. Loop Closure Parameter Tuning *
    - During testing, we found that MORB_SLAM would often correctly detect that it had formed a loop, but the system would consider it a "Bad Loop" since the calculated post-loop-close pose's rotation error was greater than the thresholds set by the system. In order to reduce the number of these false negatives for loop detection, we increased the rotation error thresholds. The previous thresholds were 0.46° for pitch/roll, and 20° for yaw. After testing, they were updated to 1.8° for pitch/roll, and 360° for yaw.
    - These changes were effective in reducing the number of false negatives, however it also increased the number of false positives. This was especially noticible in the cases where there were few MapPoint matches for the KeyFrame when a loop was detected. Therefore we increased the minimum number of MapPoints required for loop to be detected by a KeyFrame from 80 to 100. To be more confident that the MapPoints matches were correct, the nnratio of the loop closing ORB matcher was reduced from 0.9 to 0.8. This means that the best MapPoint match now needs to be 20% lower than the second best match, as opposed to the previous 10%.
    - With these changes, testing showed that the odds of a false negative went down from ~20% of loops to <1%. There was no noticable change in the rate of false positives.

4. Track Local Map Parameter Tuning
    - TrackLocalMap now requires 150 MapPoint matches when the tracking state is Recently Lost, as opposed to its previous value of 10. This is because the system would often fly off in a random direction when there were this few MapPoint matches. Testing showed that the case where this threshold was too high would only cause a new map to be made when lost, which had little impact on performance, but the value being too low would cause the system to fly off. Therefore we set to to be a very large value to minimize the error created by getting lost.

5. External Map Viewer

6. Allow the Tracking State to be Manually Set to Lost (BONK)
    - There was no system in place to reset MORB-SLAM's tracking without deleting the current map. By implementing this change, we allow the user to tell MORB-SLAM to create a new map when they detect that the tracking is inconsistent with the physical motion of the camera.

7. Prevent Map Teleportation *

8. KeyFrame Culling Optimization **

9. Inertial Bundle Adjustment System Overhaul ***

10. Added the FastIMUInit and StationaryIMUInit Options to Settings
    - Enabling FastIMUInit allows the system to be initiallized faster than without it. This is useful in cases where you need less downtime between starting the system and reporting its first Pose.
    - Enabling StationaryIMUInit disables the requirement of the camera being in motion to initialize the IMU. This makes it easier to use MORB-SLAM on devices that aren't always in motion.

11. Changed Vector Normalization 
    - In certain functions a vector norm was approximated by taking the average of a group of unit vectors, meaning it did not always have a magnitude of 1. Taking the actual norm of a vector is a very inexpensive function computationally, so we increased our accuracy by calculating the norm exactly

12. Converted all Pointers to Shared Pointers **

13. Implemented Relocalization on Map Load ***

14. Moved IMU Processing Out of MORB-SLAM **

15. Merging IMU Measurements Optimization
    - Modify the Merge IMU Measurements function, changing the runtime from O(n) to O(1)