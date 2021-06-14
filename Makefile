.ONESHELL:
SHELL=/bin/bash

build:
	source /opt/ros/melodic/setup.bash
	echo $(ROS_PACKAGE_PATH)
	echo $(PYTHONPATH)
	cd ../ORB_SLAM2
	export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/ORB_SLAM2/Examples/ROS >> ~/.bashrc
	cat ~/.bashrc
	sudo chmod +x build_ros.sh
	./build_ros.sh

.PHONY: build

