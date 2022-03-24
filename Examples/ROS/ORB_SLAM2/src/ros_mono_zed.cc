/**
* ROS node to save Frametrajectory from the messages captured by ZED camera
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ros/ros.h>
#include <rosbag/bag.h>
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

private:
    double currentTimeStamp;
    double frstProcessingTimeStamp;
    double lstProcessingTimeStamp;
    std::string dataFolderPath;
    tf::Transform prevPose;

public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, double frstTimeStamp, double lstTimeStamp, std::string dataFolderPath):mpSLAM(pSLAM){
        setFrstProcessingTimeStamp(frstTimeStamp);
        setLstProcessingTimeStamp(lstTimeStamp);
        this->currentTimeStamp = 0.0;
        this->dataFolderPath = dataFolderPath;
        prevPose = tf::Transform();
        prevPose.setOrigin( tf::Vector3(0.0, 0.0, 0.0));
    	tf::Quaternion q(0.0, 0.0, 0.0, 1.0);
    	prevPose.setRotation(q);
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;

    void setFrstProcessingTimeStamp(double inputTime){
        this->frstProcessingTimeStamp = inputTime;
    }

    void setLstProcessingTimeStamp(double inputTime){
        this->lstProcessingTimeStamp = inputTime;
    }

    double getCurrentTimeStamp() const{
        return this->currentTimeStamp;
    }

    double getFrstProcessingTimeStamp() const{
        return this->frstProcessingTimeStamp;
    }

     double getLstProcessingTimeStamp() const{
        return this->lstProcessingTimeStamp;
    }


};

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

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,plot);

    ImageGrabber igb(&SLAM, frstTimeStamp, lstTimeStamp, dataFolderPath);

    ROS_INFO("First processing time stamp %15f: ", igb.getFrstProcessingTimeStamp());
    ROS_INFO("last processing time stamp %15f: ", igb.getLstProcessingTimeStamp());
    ros::NodeHandle nodeHandler;

    while (ros::ok()){
        ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
        ros::spin();
    }
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

    if (cv_ptr->header.stamp.toSec() < this->frstProcessingTimeStamp)
        return;

    if (cv_ptr->header.stamp.toSec() > this->lstProcessingTimeStamp){

        mpSLAM->Shutdown();
        mpSLAM->SaveFrameTrajectory(this->dataFolderPath + "/FrameTrajectory.txt");
        mpSLAM->SaveKeyFrameTrajectoryTUM(this->dataFolderPath + "/KeyFrameTrajectory.txt");
        ros::shutdown();
        return;
    }

    this->currentTimeStamp = cv_ptr->header.stamp.toSec();
    tf::Transform transform;
    static tf::TransformBroadcaster br;
    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec()).clone();
    if(Tcw.empty())
        transform = this->prevPose;

    else{
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    transform.setOrigin( tf::Vector3(tcw.at<float>(0), tcw.at<float>(1), tcw.at<float>(2)));
    vector<float> r = ORB_SLAM2::Converter::toQuaternion(Tcw.rowRange(0,3).colRange(0,3));
    tf::Quaternion q(r[0], r[1], r[2], r[3]);
    transform.setRotation(q);
    }
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", "world")); // camera is the parent frame
}


