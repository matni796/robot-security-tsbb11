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
#include <pthread.h>
#include <queue>
#include <tf/transform_listener.h>
//#include "floattypes.h";

#include <unistd.h>


typedef std::pair<cv::Mat, cv::Mat> elem;
bool comparePair(elem const & lhs, elem const & rhs) {
	return lhs.second.at<double>(0) < rhs.second.at<double>(0);
}

class Calibration {
public:
	bool completed;
	tf::TransformBroadcaster br;
	tf::TransformListener listener;

	std::vector<cv::Point3f> chessBoard;
	cv::Size boardLayout;
	double boardSideLength;

	std::vector<cv::Point3f> circleGrid;
	cv::Size gridLayout;
	double gridSideLength;

	std::vector<cv::Point2f> gridCorners, boardCorners;

	cv::Mat intrinsics, distortions;
	cv::Size patternSize;
	cv::Mat rvec, tvec;
	size_t maxHistorySize;
	pthread_mutex_t calibrationLock;

	void initialize(cv::Size boardLayout, double boardSideLength, cv::Size gridLayout, double gridSideLength, cv::Mat intrinsics, cv::Mat distortions, size_t maxHistorySize = 1) {
		this->boardLayout = boardLayout;
		this->gridLayout = gridLayout;

		this->intrinsics = intrinsics;
		this->distortions = distortions;
		this->maxHistorySize = maxHistorySize;
		tvec = (cv::Mat_<double>(1,3) << 0.9, -1.0, 3.0);
		rvec = (cv::Mat_<double>(1,3) << -3.23, -2.17, -1.16 );
		completed = false;
		pthread_mutex_init(&calibrationLock, NULL);
		for(int j = 0; j < boardLayout.height; ++j) {
			for(int i = 0; i < boardLayout.width; ++i) {
				chessBoard.push_back(cv::Point3f(boardSideLength*i, boardSideLength*j, 0));
			}
		}

		for(int j = 0; j < gridLayout.height; ++j) {
			for(int i = 0; i < gridLayout.width; ++i) {
				circleGrid.push_back(cv::Point3f(gridSideLength*(2*i + !(j&1)), gridSideLength*j, 0));
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
		
		bool boardOk = findChessboardCorners(cv_ptr->image, boardLayout, boardCorners, CV_CALIB_CB_ADAPTIVE_THRESH);
		bool gridOk = findCirclesGrid(cv_ptr->image, gridLayout, gridCorners, cv::CALIB_CB_ASYMMETRIC_GRID);
		ROS_INFO("%s chessboard corners; %s circle grid",
				 boardOk ? "Found" : "Not found",
				 gridOk ? "Found" : "Not found");
		
		if(boardOk) {// && gridOk) {	
			ROS_INFO("Origin to end pixel coordinates: %.1fx%.1f -> %.1fx%.1f", boardCorners.front().x, boardCorners.front().y, boardCorners.back().x, boardCorners.back().y);
			cv::Mat r,t;

			std::vector<cv::Point3f> p3p3dPoints;
			std::vector<cv::Point2f> p3p2dPoints;
			int w = boardLayout.width, h = boardLayout.height;
			int cornerIndices[] = { 0, w-1, w*(h-1)-1, w*h -1 };
			for(int i = 0; i < 4; ++i) {
				p3p3dPoints.push_back(chessBoard[cornerIndices[i]]);
				p3p2dPoints.push_back(boardCorners[cornerIndices[i]]);
			}
			cv::solvePnP(p3p3dPoints, p3p2dPoints, intrinsics, distortions, r, t, false, CV_P3P);
			
			// transformT = listener.getTransform("circle_grid", "chessboard");
			// points = boardPoints + transformT * circlePoints
			// corners = boardCorners + circleCorners
			// solvePnp(points, corners, intrinsics, distortions, r, t, false);
			cv::solvePnP(chessBoard, boardCorners, intrinsics, distortions, r, t, true, CV_ITERATIVE);

			filterTransform(r, t);
		}
		sleep(1);
	}

	std::vector<elem> history;
	std::vector<elem> sortedHistory;
	void filterTransform(cv::Mat r, cv::Mat t) {
		pthread_mutex_lock(&calibrationLock);

		history.push_back(std::make_pair(t, r));
		if(history.size() > maxHistorySize)
			history.erase(history.begin());

		sortedHistory = history;
		std::sort(sortedHistory.begin(), sortedHistory.end(), comparePair);
		tvec = sortedHistory[sortedHistory.size() / 2].first;
		rvec = sortedHistory[sortedHistory.size() / 2].second;

		pthread_mutex_unlock(&calibrationLock);
	}

	template<typename T>
	tf::Vector3 tfVectorFromCvMat(cv::Mat in) {
		return tf::Vector3(in.at<T>(0), in.at<T>(1), in.at<T>(2));
	}

	void publishTransform() {
		pthread_mutex_lock(&calibrationLock);

		if(history.size() == 0) {
			pthread_mutex_unlock(&calibrationLock);
			return;
		}

		tf::Vector3 translation = tfVectorFromCvMat<double>(tvec);
		tf::Vector3 axis = tfVectorFromCvMat<double>(rvec);
		double angle = axis.length();

		tf::Transform transform;
		transform.setOrigin(translation);
		transform.setRotation(tf::Quaternion(axis.normalize(), angle));

		ROS_INFO("Publishing t = [ %5.2f, %5.2f, %5.2f ] m, r = [ %5.2f, %5.2f, %5.2f ], %5.2f rad",
				 translation.getX(), translation.getY(), translation.getZ(),
				 axis.getX(), axis.getY(), axis.getZ(), angle);

		pthread_mutex_unlock(&calibrationLock);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_rgb_optical_frame", "pattern")); 
	}
private:
	
};

Calibration * cal;

void * publishTransform(void * in) {
	while(!cal->completed) {
		usleep(2e5);
		cal->publishTransform();
	}
}

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
	printf("camera_calibration_parsers\n");
	camera_calibration_parsers::readCalibration( getEnvVar("CAMERA_MATRIX_PATH")+"/camMatRGB.yaml", cameraName, camInfo);

	float fx = camInfo.P.elems[0];
	float cx = camInfo.P.elems[2];
	float fy = camInfo.P.elems[5];
	float cy = camInfo.P.elems[6];

	cv::Mat intrinsics = (cv::Mat_<double>(3,3) << fx, 0 ,cx , 0, fy, cy, 0, 0, 1);
	cv::Mat distortion = (cv::Mat_<double>(1,5) << camInfo.D[0], camInfo.D[1],
						  camInfo.D[2], camInfo.D[3], camInfo.D[4]);

	ros::init (argc, argv, "calibration");
	cal = new Calibration();
	cal->initialize(cv::Size(8,6), 0.058, cv::Size(3,5), 0.056, intrinsics, distortion);
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe(std::string("/camera/rgb/image_mono"), 1, &Calibration::imageReceivedCallback, cal);
	pthread_t thread;
	pthread_create(&thread, NULL, publishTransform, NULL);
	ros::spin();
	cal->completed = true;
	pthread_join(thread, NULL);
	delete cal;
}

