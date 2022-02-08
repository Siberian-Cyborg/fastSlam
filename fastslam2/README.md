# Fast slam

Implementation of [1] modified for db019/db020. Uses cones in local frames (see custom\_msgs) and velocities (nav\_msgs/Odometry). Will output map + odometry.

[1] Montemerlo, M., Thrun, S., Koller, D. and Wegbreit, B., 2002. FastSLAM: A factored solution to the simultaneous localization and mapping problem.

## Getting Started

### Prerequisites

```
ros-kinetic-eigen pcl_ros some_other_package_i_did_not_mention
```

### Building

```
catkin build fastslam
```

## Running
Get a rosbag of cone data or raw sensor data and try it out! Make sure to check topic names.
If no cones are in the bag, you will need cone perception running.

### Normal

```
roslaunch fastslam fastslam.launch
```

### Run with rosbag
Exit fastslam when rosbag is finished

```
roslaunch fastslam demo_fastslam.launch bag_filename:=${FULL_PATH_TO_BAG}
```

### Evaluation script
Check eval.bash in  evaluation\_scripts to see how this works. 
[evo](https://github.com/MichaelGrupp/evo) is required.

```
SOURCE_PATH=~/catkin_ws/ BAG_PATH=~/rosbags_db020/ bash eval.bash
```

## Running the tests

There are unit tests in the test folder. Currently there's only a dummy test.
```
catkin run_tests fastslam
catkin_test_results (catkin_ws)/build/fastslam/
```

For some reason with `catkin run_tests` failing tests will not result in an non-zero exit code???
So you need to run the second command to actually see if something failed or not.
TODO():investigate

## Authors / Contact people

* **Ahmed Agha** - *Initial version*
* **Stan Guerassimov** - *Maintainer 2019/2020*

## Acknowledgments

* based on: https://github.com/snmnmin12/fastslam/
* tested in 2019 also in FSSim


