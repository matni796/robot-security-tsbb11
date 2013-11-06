//Class for handeling distances between objects and robot
#ifndef DISTANCE_HANDLER_HPP
#define DISTSNCE_HANDLER_HPP

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

//template <>
class DistanceHandler{
private:
	cv::Mat rotationMatrix;
	ros::Subscriber calibrationSubscriber, robotSubscriber, clusteringSubsriber;



public:
	DistanceHandler(ros::NodeHandle nh){
		calibrationSubscriber = nh.subscribe("calibration_data", 1, &DistanceHandler::calibrationCallback, this);
		//TODO add robotSubsriber
		clusteringSubsriber = nh.subscribe("distance_calc", 1,  &DistanceHandler::distanceCallback, this);
	}
	void calibrationCallback(const std_msgs::Float64MultiArray& msg);
	void distanceCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif
