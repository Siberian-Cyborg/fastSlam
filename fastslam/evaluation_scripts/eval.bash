#!/bin/bash

# fast slam evaluation script
# requires: evo
clear

if [ -z "$SOURCE_PATH" ]; then
	echo "\$SOURCE_PATH not set"
	exit
else
	if [ ! -d "$SOURCE_PATH" ]; then
		echo "\$SOURCE_PATH: $SOURCE_PATH does not exist!"
		exit
	fi
fi

cd $(mktemp -d)
# 1. select dataset
strings=(
	"2020-02-23-17-18-22.bag"
	"2020-02-23-17-34-28.bag"
	)
path="${BAG_PATH}/"
allRMSE=()
allstd=()
allDatasets=()
allCones=()
for i in "${strings[@]}"; do
	for n in {0..20}; do
		# 2. run fast slam with rosbag recording gt pose + slam pose
		source $SOURCE_PATH/devel/setup.bash;
		roslaunch fastslam demo_fastslam.launch bag_filename:=$path$i record:=true record_filename:=$(pwd)/$i-valid-$n &
		wait
		# TODO remove hard coded paths
		evo_traj bag $i-valid-$n.bag /fs_odom --ref /gps/odom -a --transform_right /home/guerassim/dv_ws/src/fastslam/evaluation_scripts/offset.json --save_as_tum
		# 3. run evo tool for rmse, then std, the print out
		evo_rpe tum odom.tum fs_odom.tum -a
		rmse=`evo_rpe tum odom.tum fs_odom.tum -a | awk '/rmse/ {print $2}'`;
		max=`evo_rpe tum odom.tum fs_odom.tum -a | awk '/max/ {print $2}'`;
		rm odom.tum
		rm fs_odom.tum
		coneCount=$(head -n 1 /home/guerassim/landmarks.txt)
		allRMSE+=($rmse)
		allstd+=($max)
		allDatasets+=("$i-$n")
		allCones+=($coneCount)
		# 4. return rmse values
		echo "${i} result: rmse, max - ${rmse} ${max}";
		echo "results in ${i}-valid-${n}.bag"
		echo "$(pwd)"
	done
done
OUTPUT=${OUTPUTEVO:=result} 
# print the list 
for index in ${!allDatasets[@]}; do
	echo "${allDatasets[$index]} result: rmse ${allRMSE[$index]} std ${allstd[$index]} cones ${allCones[$index]}";
	echo "${allDatasets[$index]} result: rmse ${allRMSE[$index]} std ${allstd[$index]} cones ${allCones[$index]}" >> ${OUTPUT}.txt;
done
