.ONESHELL:
SHELL := /bin/bash

# Define paths
ROS_SETUP := /opt/ros/noetic/setup.bash
ORB_SLAM_DIR := ../ORB_SLAM2
ROS_EXAMPLES_DIR := /home/ORB_SLAM2/Examples/ROS

.PHONY: build
build:
	@echo "Building ORB-SLAM2 ROS package..."
	# Load ROS environment
	source $(ROS_SETUP)

	# Show environment info
	@echo "ROS_PACKAGE_PATH before: $$ROS_PACKAGE_PATH"
	@echo "PYTHONPATH before: $$PYTHONPATH"

	# Temporarily extend ROS_PACKAGE_PATH (no need to persist in ~/.bashrc)
	export ROS_PACKAGE_PATH="$$ROS_PACKAGE_PATH:$(ROS_EXAMPLES_DIR)"

	# Show updated path
	@echo "ROS_PACKAGE_PATH after: $$ROS_PACKAGE_PATH"

	# Move to ORB-SLAM2 directory and build
	cd $(ORB_SLAM_DIR)
	chmod +x build_ros.sh
	./build_ros.sh

	@echo "✅ Build complete."


