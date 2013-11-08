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
#include <image_transport/image_transport.h>
#include <vector>
#include <std_msgs/Float64MultiArray.h>

#include "camera_calibration_parsers/parse.h"
#include "camera_calibration_parsers/parse_ini.h"
#include "camera_calibration_parsers/parse_yml.h"
//#include "floattypes.h";

using namespace std;

ros::Publisher pub;
string fileName;
cv_bridge::CvImagePtr cv_ptr;
cv::Mat intrinsics, distortion;
vector<cv::Point3f> boardPoints;
vector<cv::Point2f> corners;
cv::Size patternsize(8,6);
string cameraName;
sensor_msgs::CameraInfo camInfo;
std_msgs::Float64MultiArray calibrationData;

std::string getEnvVar( std::string const & key ) {
  char * val;
  val = getenv( key.c_str() );
  std::string retval = "";
  if (val != NULL) {
    retval = val;
  } else {
	  cerr << "Warning! Environmentvariable " << key << " is not set!";
  }
  return retval;
}

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
	cv::Mat rvec(3,1,cv::DataType<float>::type);
	cv::Mat tvec(3,1,cv::DataType<float>::type);

	if (corners.size() == 48){
		cv::solvePnP(boardPoints, corners, intrinsics, distortion, rvec, tvec, false);

		calibrationData.layout.dim.resize(2);
		calibrationData.layout.dim[0].label = "vectors";
		calibrationData.layout.dim[0].size = 2;
		calibrationData.layout.dim[0].stride = 2*3;
		calibrationData.layout.dim[1].label = "elements";
		calibrationData.layout.dim[1].size = 3;
		calibrationData.layout.dim[1].stride = 3;

		calibrationData.data.resize(6);

		for (int i=0;i<3;i++){
			calibrationData.data[i+3] = tvec.at<double>(i,0);
			calibrationData.data[i] = rvec.at<double>(i,0);
		}
		pub.publish(calibrationData);
	}

}


int main (int argc, char** argv)
{
	camera_calibration_parsers::readCalibration( getEnvVar("CAMERA_MATRIX_PATH")+"/camMatRGB.yaml", cameraName, camInfo);

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
	pub = nh.advertise<std_msgs::Float64MultiArray>("calibration_data",1);
	ros::spin ();
}

