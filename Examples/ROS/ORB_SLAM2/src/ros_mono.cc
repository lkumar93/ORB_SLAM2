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

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include<sensor_msgs/Imu.h>
//#include<tf>
//#include <geometry_msgs/PoseStamped>
//#include <nav_msgs/Odometry>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>


ros::Publisher camera_pose_pub;

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/crazyflie/cameras/bottom/image", 1, &ImageGrabber::GrabImage,&igb);
    camera_pose_pub = nodeHandler.advertise<sensor_msgs::Imu>("/crazyflie/cameras/bottom/pose",1);
    ros::spin();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

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

    cv::Mat mTcw,Rwc,twc;

    try
    {
     mTcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
     Rwc = mTcw.rowRange(0,3).colRange(0,3).t();
    twc = -Rwc*mTcw.rowRange(0,3).col(3);
    tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
    Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
    Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
    tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

    tf::Transform tfTcw(M,V);

    tf::Quaternion Q = tfTcw.getRotation();

    sensor_msgs::Imu camera_pose;

    camera_pose.header.stamp = ros::Time::now() ;
    camera_pose.linear_acceleration.x = V.x();
    camera_pose.linear_acceleration.y = V.y();
    camera_pose.linear_acceleration.z = V.z();
    camera_pose.orientation.x = Q.getX();
    camera_pose.orientation.y = Q.getY();
    camera_pose.orientation.z = Q.getZ();
    camera_pose.orientation.w = Q.getW();
    camera_pose_pub.publish(camera_pose);

     
    }

    catch(...)
    {

	ROS_INFO(" ERROR PUBLISHING");
    }

   






}


