/*
 * calibration.cpp
 *
 *  Created on: Oct 17, 2013
 *      Author: olle
 */

#include <ros/ros.h>
// PCL specific includes
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stereo_msgs/DisparityImage.h>
#include <vector>

#include "camera_calibration_parsers/parse.h"
#include "camera_calibration_parsers/parse_ini.h"
#include "camera_calibration_parsers/parse_yml.h"

using namespace std;
ros::Publisher pub;
string filename;
void calibrate(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, "");
  	}
  	catch (cv_bridge::Exception& e)
  	{
    		ROS_ERROR("cv_bridge exception: %s", e.what());
    		return;
  	}
  	vector<cv::Point2f> corners;
  	cv::Size patternsize(8,6);
  	bool found = findChessboardCorners(cv_ptr->image,patternsize,corners, CV_CALIB_CB_ADAPTIVE_THRESH);
  	cv::FileStorage fs(filename, cv::FileStorage::READ);
  	cv::Mat intrinsics, distortion;
  	fs["camera_matrix"] >> intrinsics;
  	fs["distortion_coefficients"] >> distortion;
  	vector<cv::Point3f> boardPoints;
  	cv::Mat rvec(3,1,cv::DataType<double>::type);
  	cv::Mat tvec(3,1,cv::DataType<double>::type);
  	// fill the array

  	cv::solvePnP(cv::Mat(boardPoints), cv::Mat(corners), intrinsics,
  	                     distortion, rvec, tvec, false);
  	cout << rvec << endl << endl << tvec << endl << endlgit;
}


int main (int argc, char** argv)
{
	ros::init (argc, argv, "calibration");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/camera/rgb/image_mono", 1, calibrate);
	ros::spin ();
}

