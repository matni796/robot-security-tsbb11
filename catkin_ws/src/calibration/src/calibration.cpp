/*
 * calibration.cpp
 *
 *	Created on: Oct 17, 2013
 *		Author: olle
 */

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include "camera_calibration_parsers/parse.h"
#include "camera_calibration_parsers/parse_ini.h"
#include "camera_calibration_parsers/parse_yml.h"
//#include "floattypes.h";

#include <unistd.h>

class Calibration {
private:
	tf::TransformBroadcaster br;
	std::vector<cv::Point3f> boardPoints;
	std::vector<cv::Point2f> corners;
	cv::Mat intrinsics, distortions;
	cv::Size patternSize;
public:
	Calibration(double sideLength, cv::Size patternSize, cv::Mat intrinsics, cv::Mat distortions)
		: patternSize(patternSize), intrinsics(intrinsics), distortions(distortions) {
		for(int i = 0; i < patternSize.width; ++i) {
			for(int j = 0; j < patternSize.height; ++j) {
				boardPoints.push_back(cv::Point3f(sideLength*i, sideLength*j, 0));
			}
		}
	}

	void imageReceivedCallback(sensor_msgs::ImageConstPtr const & msg) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, "");
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("Error while trying to interpret image message with cv_bridge: %s", e.what());
			return;
		}
		
		bool success = findChessboardCorners(cv_ptr->image, patternSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH);
		int expectedCorners = patternSize.width * patternSize.height;
		ROS_INFO("%s at %2d / %2d chessboard corners", success ? "Found" : "Not found", corners.size(), expectedCorners);
		if(success && corners.size() == expectedCorners) {
			cv::Mat rvec, tvec;
			cv::solvePnP(boardPoints, corners, intrinsics, distortions, rvec, tvec, false);

			tf::Transform transform;
			transform.setOrigin(tf::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));
			transform.setRotation(tf::Quaternion(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)));
			ROS_INFO("Publishing t = [ %4.1f, %4.1f, %4.1f ] m, r = [ %4.1f, %4.1f, %4.1f ] rad",
					 tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2),
					 rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_rgb_optical_frame", "pattern")); 
		}
	}
private:
	
};

using namespace std;

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


int main (int argc, char** argv) {
	sensor_msgs::CameraInfo camInfo;
	std::string cameraMatrixPath = getEnvVar("CAMERA_MATRIX_PATH"), cameraName;
	camera_calibration_parsers::readCalibration( getEnvVar("CAMERA_MATRIX_PATH")+"/camMatRGB.yaml", cameraName, camInfo);

	float fx = camInfo.P.elems[0];
	float cx = camInfo.P.elems[2];
	float fy = camInfo.P.elems[5];
	float cy = camInfo.P.elems[6];

	cv::Mat intrinsics = (cv::Mat_<double>(3,3) << fx, 0 ,cx , 0, fy, cy, 0, 0, 1);
	cv::Mat distortion = (cv::Mat_<double>(1,5) << camInfo.D[0], camInfo.D[1],
						  camInfo.D[2], camInfo.D[3], camInfo.D[4]);

	ros::init (argc, argv, "calibration");
	Calibration cal(0.04, cv::Size(8, 6), intrinsics, distortion);
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe(std::string("/camera/rgb/image_mono"), 1, &Calibration::imageReceivedCallback, &cal);
	ros::spin();
}

