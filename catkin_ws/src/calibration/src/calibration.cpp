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
#include <vector>

#include "camera_calibration_parsers/parse.h"
#include "camera_calibration_parsers/parse_ini.h"
#include "camera_calibration_parsers/parse_yml.h"

using namespace std;

ros::Publisher pub;
string fileName = "/home/matt/dev/robot-security-tsbb11/catkin_ws/camMatRGB.yaml";
cv_bridge::CvImagePtr cv_ptr;
cv::Mat intrinsics, distortion;
vector<cv::Point3f> boardPoints;
vector<cv::Point2f> corners;
cv::Size patternsize(8,6);
string cameraName;
sensor_msgs::CameraInfo camInfo;

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

  	bool found = findChessboardCorners(cv_ptr->image,patternsize,corners, CV_CALIB_CB_ADAPTIVE_THRESH);
  	cv::Mat rvec(3,1,cv::DataType<double>::type);
  	cv::Mat tvec(3,1,cv::DataType<double>::type);

  	if (corners.size() == 48){
  		cv::solvePnP(boardPoints, corners, intrinsics, distortion, rvec, tvec, false);
  		cout << rvec << endl << endl << tvec << endl << endl;
  	}


}


int main (int argc, char** argv)
{
	camera_calibration_parsers::readCalibration(fileName, cameraName, camInfo);

	float fx = camInfo.P.elems[0];
	float cx = camInfo.P.elems[2];
	float fy = camInfo.P.elems[5];
	float cy = camInfo.P.elems[6];

	intrinsics = (cv::Mat_<double>(3,3) << fx, 0 ,cx , 0, fy, cy, 0, 0, 1);
	distortion = (cv::Mat_<double>(1,5) << camInfo.D[0], camInfo.D[1],
							camInfo.D[2], camInfo.D[3], camInfo.D[4]);

	for(int x= 0; x<6; ++ x){
		for(int y= 0; y<8; ++y){
			boardPoints.push_back(cv::Point3f(0.04*x,0.04*y,0.0));
		}
	}

	ros::init (argc, argv, "calibration");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/camera/rgb/image_mono", 1, calibrate);
	ros::spin ();
}

