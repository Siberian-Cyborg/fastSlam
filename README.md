# fastSlam
FastSLAM 1 and FastSLAM 2 for an autonomous race car using preprocessed Lidar point cloud as input and IMU+wheel encoders for odometry. The python script EKF_odometry2 is used to fuse the SLAM state estimation, IMU and wheel encoders and is then inputted into the SLAM as odometry data for the prediction step. The features extracted from Lidar data are inputted into the correction step.

This is just one part of the entire software stack for the autonomous car. The communication is done thru ROS.

Everything is described here:
[Documentation.pdf](https://github.com/Siberian-Cyborg/fastSlam/files/8027131/Documentation.pdf)
