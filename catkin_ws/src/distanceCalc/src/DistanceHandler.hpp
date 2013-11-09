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


struct ObjectsAndRobot{
	clustering::clusterArray objects;
	clustering::clusterArray robot;
};

//template <>
class DistanceHandler{
private:
	cv::Mat rotationMatrix;
	cv::Mat tvec;
	ros::Subscriber calibrationSubscriber, robotSubscriber, clusteringSubscriber;
	ros::Publisher distancePublisher;
	clustering::clusterArray rawClusters;
	clustering::clusterArray objectClusters;
	clustering::clusterArray robotClusters;
	float insideRobotParameter;

	//variables from distanceCalculator
	float minDistance;
	pcl::PointXYZ point;
	pcl::PointXYZ p;
	std::vector<cv::Mat> robotJoint;
	std::vector<float> sqrLengthBetweenJoints;
	std::vector<float> radiusOfCylinders;
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
		//only for testing
		robotJoint.push_back(cv::Mat_<float>(3,1) <<0.0f,-0.5f,1.5f);
		robotJoint.push_back(cv::Mat_<float>(3,1) <<0.0f,0.5f,1.5f);
		sqrLengthBetweenJoints.push_back(0.1f);
		radiusOfCylinders.push_back(3.0f);
		insideRobotParameter = 0.01;
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

	///Functions regarding calculation of distances
	void distanceCallback(clustering::clusterArray msg){
		std::cout << "received clusters" << std::endl;
		rawClusters = msg;
		ObjectsAndRobot result = removeRobot(msg); //TODO not implemented
		objectClusters = result.objects;
		robotClusters = result.robot;
		distanceCalc(objectClusters);

	}

	void compareToRobot(float x, float y, float z){


		for(int i = 0; i < robotJoint.size(); i++){
			float distance = powf(robotJoint[i].at<float>(0,0)-x, 2)+powf(robotJoint[i].at<float>(1,0)-y, 2)+powf(robotJoint[i].at<float>(2,0)-z, 2);
			if (distance <= minDistance)
			{
				minDistance = distance;
				minPoint.x = x;
				minPoint.y = y;
				minPoint.z = z;
				closestJoint = robotJoint[i]; //should produce errors
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

	//Functions regarding removal of the robot!
	ObjectsAndRobot removeRobot(clustering::clusterArray& rawClusterCloud){
		clustering::clusterArray objectArray;
		clustering::clusterArray robotArray;

		int numberOfClusters=0;
		int removedClusters = 0;
		for(int i=0; i<rawClusterCloud.ca.size(); i++)
		{
			++numberOfClusters;
			float inside = 0;
			float outside = 0;
			for(int j=0; j<rawClusterCloud.ca[i].pa.size(); j++)
			{
				if(pointInsideRobot(rawClusterCloud.ca[i].pa[j]))
					++inside;
				else
					++outside;
			}

			if((inside+outside != 0)&&!(inside/(inside+outside) > insideRobotParameter) ){
				objectArray.ca.push_back(rawClusterCloud.ca[i]); // add cluster if not inside robot

			} else{
				robotArray.ca.push_back(rawClusterCloud.ca[i]);// add cluster if inside robot
				++removedClusters;
			}
			std::cout << "inside: "<< inside <<std::endl;
			std::cout << "outside: "<< outside <<std::endl;

		}

		std::cout << "numberofclusters: " << numberOfClusters <<std::endl;
		std::cout << "removedClusters: " << removedClusters <<std::endl;

		return ObjectsAndRobot{objectArray, robotArray};

	}

	bool pointInsideRobot(clustering::point p){
		for(int i=0; i<robotJoint.size()-1; i++){
			if (rotationMatrix){
				cv::Mat joint1KinectCoord = rotationMatrix*robotJoint[i]+tvec;
				cv::Mat joint2KinectCoord = rotationMatrix*robotJoint[i+1]+tvec;

				if (pointInsideCylinder(joint1KinectCoord,joint2KinectCoord,sqrLengthBetweenJoints[i],radiusOfCylinders[i],p))
							return true;
			} // rotationMatrix*robotJoint[i] +tvec,rotationMatrix*robotJoint[i+1] +tvec,sqrLengthBetweenJoints[i],radiusOfCylinders[i],p
			if (pointInsideCylinder(robotJoint[i],robotJoint[i+1],sqrLengthBetweenJoints[i],radiusOfCylinders[i],p))
				return true;
		}
		return false;
	}

	//Algorithm copied from http://www.flipcode.com/archives/Fast_Point-In-Cylinder_Test.shtml
	bool pointInsideCylinder( cv::Mat& pt1, cv::Mat& pt2, float lengthsq, float radius_sq, const clustering::point& testpt )
	{
		float dx, dy, dz;	// vector d  from line segment point 1 to point 2
		float pdx, pdy, pdz;	// vector pd from point 1 to test point
		float dot, dsq;

		dx = pt2.x - pt1.x;	// translate so pt1 is origin.  Make vector from
		dy = pt2.y - pt1.y;     // pt1 to pt2.  Need for this is easily eliminated
		dz = pt2.z - pt1.z;

		pdx = testpt.x - pt1.x;		// vector from pt1 to test point.
		pdy = testpt.y - pt1.y;
		pdz = testpt.z - pt1.z;


		dot = pdx * dx + pdy * dy + pdz * dz;


		if( dot < 0.0f || dot > lengthsq )
		{
			//std::cout << "beside" << std::endl;
			return(false);
		}
		else
		{

			dsq = (pdx*pdx + pdy*pdy + pdz*pdz) - dot*dot/lengthsq;

			if( dsq > radius_sq )
			{
				//std::cout << "outside" << std::endl;
				return(false);
			}
			else
			{
				//std::cout << "inside" << std::endl;
				return(true);		// return true if inside cylinder
			}
		}
	}
};


#endif
