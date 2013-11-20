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
#include <tf/transform_broadcaster.h>
#include "camera_calibration_parsers/parse.h"
#include "camera_calibration_parsers/parse_ini.h"
#include "camera_calibration_parsers/parse_yml.h"
//#include "floattypes.h";

#include <unistd.h>

using namespace std;

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
    }
    else {
		cerr << "Warning! Environmentvariable " << key << " is not set!";
    }
    return retval;
}

void calibrate(const sensor_msgs::ImageConstPtr& msg)
{
	static tf::TransformBroadcaster br;
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

	if (corners.size() == 48) {
		cv::solvePnP(boardPoints, corners, intrinsics, distortion, rvec, tvec, false);
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));
		transform.setRotation(tf::Quaternion(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_rgb_optical_frame", "pattern")); 
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

    for(int y= 0; y<6; ++y){
		for(int x= 0; x<8; ++ x){
			boardPoints.push_back(cv::Point3f(0.04*x,0.04*y,0.0));
		}
    }

    ros::init (argc, argv, "calibration");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera/rgb/image_mono", 1, calibrate);
    ros::spin ();
}

