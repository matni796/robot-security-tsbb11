//Class for handeling distances between objects and robot
#ifndef DISTANCE_HANDLER_HPP
#define DISTANCE_HANDLER_HPP

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
	cv::Mat tvec;
	ros::Subscriber calibrationSubscriber, robotSubscriber, clusteringSubscriber;
	ros::Publisher distancePublisher;
	clustering::clusterArray rawClusters;
	clustering::clusterArray filteredClusters;

	//variables from distanceCalculator
	float minDistance;
	pcl::PointXYZ point;
	pcl::PointXYZ p;
	std::vector<pcl::PointXYZ> robotJoint;
	pcl::PointXYZ minPoint;
	pcl::PointXYZ closestJoint;
	//float distance;
	std_msgs::Float32MultiArray returnArray;
	int numberOfClusters;

public:
	DistanceHandler(ros::NodeHandle& nh) : tvec(3,1,cv::DataType<float>::type), robotJoint(2) {
		calibrationSubscriber = nh.subscribe("calibration_data", 1, &DistanceHandler::calibrationCallback, this);
		//TODO add robotSubsriber
		clusteringSubscriber = nh.subscribe("cluster_vectors", 1,  &DistanceHandler::distanceCallback, this);
		distancePublisher = nh.advertise<std_msgs::Float32MultiArray>("distances", 1);

	}
	void calibrationCallback(const std_msgs::Float64MultiArray& msg){
		cv::Mat rvec(3,1,cv::DataType<float>::type);
		std::cout << msg << std::endl;
		for (int i = 0; i <3; ++i){
			tvec.at<float>(i,0) = msg.data[i];
			rvec.at<float>(i,0) = msg.data[i+3];
		}
		cv::Rodrigues(rvec,rotationMatrix);

	}
	void distanceCallback(clustering::clusterArray msg){
		rawClusters = msg;
		filteredClusters = removeRobot(msg); //TODO not implemented
		distanceCalc(filteredClusters);

	}

	void compareToRobot(float x, float y, float z){


		for(int i = 0; i < robotJoint.size(); i++){
			float distance = powf(robotJoint[i].x-x, 2)+powf(robotJoint[i].y-y, 2)+powf(robotJoint[i].z-z, 2);
			if (distance <= minDistance)
			{
				minDistance = distance;
				minPoint.x = x;
				minPoint.y = y;
				minPoint.z = z;
				closestJoint = robotJoint[i];
			}
		}
	}
	void distanceCalc(clustering::clusterArray clusters){


		numberOfClusters = clusters.ca.size();
		returnArray.layout.dim.resize(2);
		returnArray.data.resize(numberOfClusters*7);
		returnArray.layout.dim[0].size = 7;
		returnArray.layout.dim[0].stride = 7*numberOfClusters;
		returnArray.layout.dim[1].size = numberOfClusters;
		returnArray.layout.dim[1].stride = 7;
//		cout << "Size of clusterArray (outer) " << endl;
//		cout << clusters.ca.size() << endl;

		for(int j = 0; j<clusters.ca.size();j++){
			minDistance = 10000.0f;
			for(int i = 0; i<clusters.ca[j].pa.size(); i++){
				compareToRobot(clusters.ca[j].pa[i].x, clusters.ca[j].pa[i].y, clusters.ca[j].pa[i].z);
			}
			returnArray.data[0+j*7] = minPoint.x;
			returnArray.data[1+j*7] = minPoint.x;
			returnArray.data[2+j*7] = minPoint.x;
			returnArray.data[3+j*7] = closestJoint.x;
			returnArray.data[4+j*7] = closestJoint.y;
			returnArray.data[5+j*7] = closestJoint.z;
			returnArray.data[6+j*7] = minDistance;
			//ROS_INFO("%f",minDistance);
//			cout << "Printing mindistance:" << endl;
//			cout << "MINDISTANCE" << endl;
//			cout << returnArray.data[6+j*7] << endl;
//			cout << ("%f",minDistance)	<< endl;
		}
		//cout << "Publishibng returnArray..." << endl;
		distancePublisher.publish(returnArray);
	}

	clustering::clusterArray_ removeRobot(clustering::clusterArray rawClusterCloud){

	}

};

#endif
