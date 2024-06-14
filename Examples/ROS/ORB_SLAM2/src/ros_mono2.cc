/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <rosbag/bag.h>
// #include <ros/console.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include "opencv2/ccalib/omnidir.hpp"

#include "System.h"
#include "Converter.h"
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

#include <algorithm> //boolalpha

using namespace std;

class ImageGrabber
{
public:
	ImageGrabber(ORB_SLAM2::System *pSLAM) : mpSLAM(pSLAM) {}

	void GrabImage(const sensor_msgs::ImageConstPtr &msg);
	tf::Transform GrabImage(cv::Mat &frame, tf::Transform &pose_previous, double &time_stamp);

	ORB_SLAM2::System *mpSLAM;
};

nav_msgs::Odometry create_odometry_message(tf::Transform &transform, ros::Time &time_stamp)
{
	// creates a ros odometry message

	nav_msgs::Odometry msg_odom;
	msg_odom.header.stamp = time_stamp;
	msg_odom.header.frame_id = "odom";
	msg_odom.header.seq = 0;
	msg_odom.child_frame_id = "odom_link";

	// msg_odom.pose.pose.position = transform.getOrigin();
	msg_odom.pose.pose.position.x = transform.getOrigin().getX();
	msg_odom.pose.pose.position.y = transform.getOrigin().getY();
	msg_odom.pose.pose.position.z = transform.getOrigin().getZ();

	// msg_odom.pose.pose.orientation = transform.getRotation();
	// normalize ()
	msg_odom.pose.pose.orientation.x = transform.getRotation().x();
	msg_odom.pose.pose.orientation.y = transform.getRotation().y();
	msg_odom.pose.pose.orientation.z = transform.getRotation().z();
	msg_odom.pose.pose.orientation.w = transform.getRotation().w();

	/*msg_odom.pose.covariance[0] = std::pow(ins_data->PosUncertainty,2.0);
	msg_odom.pose.covariance[7] = std::pow(ins_data->PosUncertainty,2.0);
	msg_odom.pose.covariance[14] = std::pow(ins_data->PosUncertainty,2.0);
	msg_odom.pose.covariance[21] = std::pow(ins_data->RollUncertainty,2.0);
	msg_odom.pose.covariance[28] = std::pow(ins_data->PitchUncertainty,2.0);
	msg_odom.pose.covariance[35] = std::pow(ins_data->YawUncertainty,2.0);*/
	return msg_odom;
}

bool to_bool(std::string str)
{
	std::transform(str.begin(), str.end(), str.begin(), ::tolower);
	std::istringstream is(str);
	bool b;
	is >> std::boolalpha >> b;
	return b;
}

enum eTrackingState
{
	SYSTEM_NOT_READY = -1,
	NO_IMAGES_YET = 0,
	NOT_INITIALIZED = 1,
	OK = 2,
	LOST = 3
};

std::ostream &operator<<(std::ostream &out, const eTrackingState value)
{
	const char *s = 0;
#define PROCESS_VAL(p) \
	case (p):          \
		s = #p;        \
		break;
	switch (value)
	{
		PROCESS_VAL(SYSTEM_NOT_READY);
		PROCESS_VAL(NO_IMAGES_YET);
		PROCESS_VAL(NOT_INITIALIZED);
		PROCESS_VAL(OK);
		PROCESS_VAL(LOST);
	}
#undef PROCESS_VAL
	return out << s;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mono");
	ros::start();

	if (argc != 4)
	{
		cerr << endl
			 << "Usage: rosrun ORB_SLAM2 Mono2 path_to_vocabulary path_to_settings slam_config_file_path" << endl;
		ros::shutdown();
		return 1;
	}
	// read config
	std::ifstream file(argv[3]);
	std::string data_folder_path, video_path, line, camera_model;

	std::getline(file, data_folder_path);
	std::getline(file, video_path);
	std::getline(file, line);
	double first_time_stamp = std::stod(line);
	std::getline(file, line);
	double fps = std::stod(line);
	double frame_duration = 1.0 / fps;
	std::getline(file, line);
	double resize_double = std::stod(line);
	std::getline(file, line);
	bool PLOT = to_bool(line);
	std::getline(file, line);
	int start_video_idx = std::stoi(line);
	std::getline(file, line);
	int end_video_idx = std::stoi(line);
	std::getline(file, line);
	int try_initialization_max_frames = std::stoi(line);
	std::getline(file, line);
	int tracking_lost_max_frames = std::stoi(line);
	std::getline(file, line);
	int tracking_lost_max_consecutive_frames = std::stoi(line);
	std::getline(file, camera_model);

	std::vector<double> frame_timestamps;
	if (camera_model == "sony")
	{
		int timestamp_index = 0;
		double current_timestamp = first_time_stamp;
		while (timestamp_index <= end_video_idx)
		{
			frame_timestamps.push_back(current_timestamp);
			current_timestamp += frame_duration;
		}
	}
	else
	{
		// split camera id from camera model
		std::stringstream camera_model_stream(camera_model);
		std::string segment;
		std::vector<std::string> segment_list;

		while (std::getline(camera_model_stream, segment, '_'))
		{
			segment_list.push_back(segment);
		}
		std::string camera_id = segment_list.back();
		std::string timestamp_path = data_folder_path + "/timestamps_" + camera_id + ".txt";
		std::fstream timestamps_file(timestamp_path);

		std::string timestamp_line;
		while (std::getline(timestamps_file, timestamp_line))
		{
			frame_timestamps.push_back(std::stod(timestamp_line));
		}
	}

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, PLOT);

	ImageGrabber igb(&SLAM);
	ROS_INFO("ORB Slam 2 created");
	ROS_INFO_STREAM("first: " << frame_timestamps[0] << " last: " << frame_timestamps.back());

	// ros::NodeHandle nodeHandler;

	// open video
	cv::Mat frame;
	int frame_idx = -1;
	cv::VideoCapture capture(video_path);
	if (!capture.isOpened())
		throw std::invalid_argument("Error when reading video");
	for (int i = 0; i < start_video_idx; i++)
	{
		capture >> frame;
		frame_idx++;
	}
	// set frame to start_video_idx
	/*bool success = capture.set(CAP_PROP_POS_FRAMES, start_video_idx);
	if (!success)
	{
		for (int i = 0; i < start_video_idx; i++)
		{
			capture >> frame;
			frame_idx++;
		}
	}
	else
	{
		frame_idx = start_video_idx;
	}*/
	double frame_time = frame_timestamps[frame_idx];
	std::string fname = data_folder_path + "/QAQC/slam_first_" + std::to_string(frame_time) + ".png";
	cv::imwrite(fname, frame);

	ROS_INFO_STREAM("ORB Slam starting frame: " << frame_idx << " subsequent frame indices are relative to this");

	eTrackingState previous_tracking_state = static_cast<eTrackingState>(igb.mpSLAM->GetTrackingState());
	ROS_INFO_STREAM("ORB Slam state: " << previous_tracking_state);

	// open ros bag
	rosbag::Bag bag;
	bag.open(data_folder_path + "/odometry.bag", rosbag::bagmode::Write);

	// variables
	bool publish = true;
	tf::Transform pose_previous;
	pose_previous.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	tf::Quaternion q(0.0, 0.0, 0.0, 1.0);
	pose_previous.setRotation(q);
	nav_msgs::Odometry odometry_msg;
	// export LD_LIBRARY_PATH=/home/catkin_ws/devel/lib:/opt/ros/noetic/lib:/home/Pangolin/build/src/:/home/ORB_SLAM2/lib/
	int nb_tracking_failed_frames = 0;
	int nb_consecutive_tracking_failed_frames = 0;
	int nb_consecutive_tracking_failed_frames_max = 0;
	int return_code = 0;
	// start tracking
	if (publish)
	{
		// ros::Publisher slam_pub = n.advertise<tf::Transform>("mono", 1000);
		while (ros::ok() && frame_idx <= end_video_idx)
		{
			// read image
			capture >> frame;
			frame_idx++;
			frame_time = frame_timestamps[frame_idx];
			if (frame.size().width < 1)
			{
				break;
			}
			// resize image
			if (resize_double != 1.0)
			{
				cv::resize(frame, frame, cv::Size(), resize_double, resize_double);
			}
			// track slam
			tf::Transform pose = igb.GrabImage(frame, pose_previous, frame_time);
			eTrackingState current_tracking_status = static_cast<eTrackingState>(igb.mpSLAM->GetTrackingState());
			if (current_tracking_status != previous_tracking_state)
			{
				ROS_INFO_STREAM("ORB Slam state: " << current_tracking_status << ", frame: " << (frame_idx - start_video_idx));
			}
			if ((frame_idx - start_video_idx) % 5000 == 0)
			{
				ROS_INFO_STREAM("ORB Slam state: " << current_tracking_status << ", frame: " << (frame_idx - start_video_idx));
			}
			if (current_tracking_status == OK && previous_tracking_state == NOT_INITIALIZED)
			{
				ROS_INFO_STREAM("ORB Slam successfully initialized on frame: " << (frame_idx - start_video_idx));
			}
			if (current_tracking_status < OK && ((frame_idx - start_video_idx) % 100 == 0))
			{
				ROS_INFO_STREAM("Trying to initialize Slam... frame: " << (frame_idx - start_video_idx));
			}
			if (current_tracking_status == LOST)
			{
				++nb_tracking_failed_frames;
				++nb_consecutive_tracking_failed_frames;
			}
			else
			{
				if (nb_consecutive_tracking_failed_frames > nb_consecutive_tracking_failed_frames_max)
				{
					nb_consecutive_tracking_failed_frames_max = nb_consecutive_tracking_failed_frames;
				}
				nb_consecutive_tracking_failed_frames = 0;
			}
			if (current_tracking_status < OK && previous_tracking_state < OK && ((frame_idx - start_video_idx) > try_initialization_max_frames))
			{
				ROS_ERROR_STREAM("Initialization failed after trying for " << try_initialization_max_frames << " frames, exiting... (frame: " << (frame_idx - start_video_idx) << ")");
				bag.close();
				// Stop all threads
				SLAM.Shutdown();
				ros::shutdown();
				return 50;
			}
			if (nb_tracking_failed_frames > tracking_lost_max_frames)
			{
				ROS_ERROR_STREAM("Tracking lost for more than " << tracking_lost_max_frames << " frames, exiting... (frame: " << (frame_idx - start_video_idx) << ")");

				return_code = 51;
				break;
			}
			if (nb_consecutive_tracking_failed_frames > tracking_lost_max_consecutive_frames)
			{
				ROS_ERROR_STREAM("Tracking lost for more than " << tracking_lost_max_consecutive_frames << " consecutive frames, exiting... (frame: " << (frame_idx - start_video_idx) << ")");
				return_code = 52;
				break;
			}

			previous_tracking_state = current_tracking_status;
			// create odometry message from pose and time stamp
			ros::Time time_stamp(frame_time);
			odometry_msg = create_odometry_message(pose, time_stamp);
			// ROS_INFO_STREAM(odometry_msg.pose.pose.position);
			bag.write("odom", time_stamp, odometry_msg);
			// Publish tf transform
			static tf::TransformBroadcaster br;
			br.sendTransform(tf::StampedTransform(pose, time_stamp, "camera", "world")); // camera is the parent frame
			// slam_pub.publish(tf_msg);
			ros::spinOnce();
		}
	}
	else
	{
		// ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
	}
	ROS_INFO("ORB Slam finished");
	if (nb_tracking_failed_frames > 0)
	{
		ROS_INFO_STREAM("Lost tracking for total of " << nb_tracking_failed_frames << " frames");
	}
	if (nb_consecutive_tracking_failed_frames_max > 0)
	{
		ROS_INFO_STREAM("Lost tracking for maximum of " << nb_consecutive_tracking_failed_frames_max << " consecutive frames");
	}
	// ros::spin();
	bag.close();
	// Stop all threads
	SLAM.Shutdown();

	// Save camera trajectory
	ROS_INFO("Saving frame trajectory.");
	SLAM.SaveFrameTrajectory(data_folder_path + "/FrameTrajectory.txt");
	ROS_INFO("Saving keyframe trajectory.");
	SLAM.SaveKeyFrameTrajectoryTUM(data_folder_path + "/KeyFrameTrajectory.txt");

	ros::shutdown();

	return return_code;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &msg)
{
	// Copy the ros image message to cv::Mat.
	cv_bridge::CvImageConstPtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvShare(msg);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
	cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec()).clone();
	if (Tcw.empty())
		return;
	cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
	// create transform
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(tcw.at<float>(0), tcw.at<float>(1), tcw.at<float>(2)));
	vector<float> r = ORB_SLAM2::Converter::toQuaternion(Tcw.rowRange(0, 3).colRange(0, 3));
	tf::Quaternion q(r[0], r[1], r[2], r[3]);
	transform.setRotation(q);
	// Publish tf transform
	static tf::TransformBroadcaster br;
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", "world")); // camera is the parent frame
}

tf::Transform ImageGrabber::GrabImage(cv::Mat &frame, tf::Transform &pose_previous, double &time_Stamp)
{
	// save old pose
	// pose_previous = pose;

	// slam frame
	cv::Mat Tcw = mpSLAM->TrackMonocular(frame, time_Stamp).clone();
	if (Tcw.empty())
		return pose_previous;
	cv::Mat tcw = Tcw.rowRange(0, 3).col(3);

	// ROS_INFO_STREAM(Tcw);
	// ROS_INFO_STREAM(mpSLAM->mpTracker->mCurrentFrame.mTcw);

	// Create tf transform
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(tcw.at<float>(0), tcw.at<float>(1), tcw.at<float>(2)));
	vector<float> r = ORB_SLAM2::Converter::toQuaternion(Tcw.rowRange(0, 3).colRange(0, 3));
	tf::Quaternion q(r[0], r[1], r[2], r[3]);
	transform.setRotation(q);
	return transform;
}
