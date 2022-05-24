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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <rosbag/bag.h>
//#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include "opencv2/ccalib/omnidir.hpp"

#include"System.h"
#include"Converter.h"
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

#include <algorithm> //boolalpha

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    tf::Transform GrabImage(cv::Mat& frame, tf::Transform& pose_previous, double& time_stamp);

    ORB_SLAM2::System* mpSLAM;
};

nav_msgs::Odometry create_odometry_message(tf::Transform& transform, ros::Time& time_stamp){
	//creates a ros odometry message

	nav_msgs::Odometry msg_odom;
	msg_odom.header.stamp = time_stamp;
	msg_odom.header.frame_id = "odom";
	msg_odom.header.seq = 0;
	msg_odom.child_frame_id = "odom_link";

	//msg_odom.pose.pose.position = transform.getOrigin();
	msg_odom.pose.pose.position.x = transform.getOrigin().getX();
        msg_odom.pose.pose.position.y = transform.getOrigin().getY();
        msg_odom.pose.pose.position.z = transform.getOrigin().getZ();

	//msg_odom.pose.pose.orientation = transform.getRotation();
	//normalize ()
	msg_odom.pose.pose.orientation.x = transform.getRotation().x();
	msg_odom.pose.pose.orientation.y = transform.getRotation().y();
	msg_odom.pose.pose.orientation.z = transform.getRotation().z();
	msg_odom.pose.pose.orientation.w = transform.getRotation().w();

	return msg_odom;
}

bool to_bool(std::string str) {
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    std::istringstream is(str);
    bool b;
    is >> std::boolalpha >> b;
    return b;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Mono_ZED");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    std::ifstream cfgFile(argv[3]);
    std::string dataFolderPath = "";
    std::string line = "";
    double frstTimeStamp = 0;
    double lstTimeStamp = 0;
    bool plot = false;

    std::getline(cfgFile, dataFolderPath);

    std::getline(cfgFile, line);    // reading plot flag
    plot = to_bool(line);

    std::getline(cfgFile, line);    //reading first time stamp
    frstTimeStamp = std::stod(line);

    std::getline(cfgFile, line);    //reading last time stamp
    lstTimeStamp = std::stod(line);

	// read time_stamp.txt file and save the time stamps in a vector.
	std::vector<double> time_stamps;
	std::ifstream in(dataFolderPath+"/time_stamps.txt");
	std::string str;
    // Read the next line from File untill it reaches the end.
    while (std::getline(in, str))
    {
        // Line contains string of length > 0 then save it in vector
        if(str.size() > 0)
            time_stamps.push_back(std::stod(str));
    }
    in.close()

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,PLOT);

	ImageGrabber igb(&SLAM);
	//ros::NodeHandle nodeHandler;

	//open video
	auto video_path = dataFolderPath+"/zed.avi";
	cv::Mat frame;
	int frame_idx = -1;
	cv::VideoCapture capture(video_path);
	if( !capture.isOpened() )
		throw std::invalid_argument( "Error when reading video" );
	if (capture.get(cv::CAP_PROP_FRAME_COUNT) != time_stamps.size())
	    throw std::invalid_argument("The number of video frames is not equal to the number of time stamps");

	//set frame to start_video_idx
	auto time_stamp_itr = time_stamps.begin();  // set time stamp pointer to the beginning of the time_stamps vector
	double frame_time = *time_stamp_itr/(10^9);     // value of the first time stamp in seconds

	//open ros bag
	rosbag::Bag bag;
	bag.open(data_folder_path + "/odometry.bag", rosbag::bagmode::Write);

	//variables
	bool publish=true;
	tf::Transform pose_previous;
	pose_previous.setOrigin( tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q(0.0, 0.0, 0.0, 1.0);
    pose_previous.setRotation(q);
	nav_msgs::Odometry odometry_msg;
	//start tracking
	if(publish){
		//ros::Publisher slam_pub = n.advertise<tf::Transform>("mono", 1000);
		while (ros::ok() && frame_time <= lstTimeStamp){


			//read image
			capture >> frame;

			if(frame.size().width < 1){
				break;

			}
			//resize image
			if(resize_double != 1.0){
				cv::resize(frame, frame, cv::Size(), resize_double, resize_double);
			}

			// if the current time is less than the first time stamp for processing
			if (frame_time < frstTimeStamp){
			    time_stamp_itr++;
			    frame_time = *time_stamp_itr/(10^9);
			    continue;
			}

			//track slam
			tf::Transform pose = igb.GrabImage(frame, pose_previous, frame_time);
			//create odometry message from pose and time stamp
			ros::Time time_stamp(frame_time);
			odometry_msg = create_odometry_message(pose, time_stamp);
			//ROS_INFO_STREAM(odometry_msg.pose.pose.position);
			bag.write("odom", time_stamp, odometry_msg);
			// Publish tf transform
			static tf::TransformBroadcaster br;
			br.sendTransform(tf::StampedTransform(pose, time_stamp, "camera", "world")); // camera is the parent frame
			//slam_pub.publish(tf_msg);
			ros::spinOnce();
			time_stamp_itr++;
			frame_time = *time_stamp_itr/(10^9);

		}
	}


    //ros::spin();
	bag.close();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    ROS_INFO("Saving frame trajectory.");
    SLAM.SaveFrameTrajectory(data_folder_path + "/FrameTrajectory.txt");
    ROS_INFO("Saving keyframe trajectory.");
    SLAM.SaveKeyFrameTrajectoryTUM(data_folder_path + "/KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec()).clone();
    if(Tcw.empty())
        return;
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    // create transform
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(tcw.at<float>(0), tcw.at<float>(1), tcw.at<float>(2)));
    vector<float> r = ORB_SLAM2::Converter::toQuaternion(Tcw.rowRange(0,3).colRange(0,3));
    tf::Quaternion q(r[0], r[1], r[2], r[3]);
    transform.setRotation(q);
    // Publish tf transform
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", "world")); // camera is the parent frame
}

tf::Transform ImageGrabber::GrabImage(cv::Mat& frame, tf::Transform& pose_previous, double& time_Stamp)
{

    //slam frame
    cv::Mat Tcw = mpSLAM->TrackMonocular(frame, time_Stamp).clone();
    if(Tcw.empty())
        return pose_previous;
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);

    // Create tf transform
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(tcw.at<float>(0), tcw.at<float>(1), tcw.at<float>(2)));
    vector<float> r = ORB_SLAM2::Converter::toQuaternion(Tcw.rowRange(0,3).colRange(0,3));
    tf::Quaternion q(r[0], r[1], r[2], r[3]);
    transform.setRotation(q);
    return transform;
}


