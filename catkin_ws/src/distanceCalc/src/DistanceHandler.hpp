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
#include <visualization_msgs/Marker.h>


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
	ros::Publisher distancePublisher, cylinderPublisher;
	clustering::clusterArray rawClusters;
	clustering::clusterArray objectClusters;
	clustering::clusterArray robotClusters;
	float insideRobotParameter;
	visualization_msgs::Marker cylinder;

	//variables from distanceCalculator
	float minDistance;
	pcl::PointXYZ point;
	pcl::PointXYZ p;
	std::vector<cv::Mat_ <float> > robotJoint;
	std::vector<float> sqrLengthBetweenJoints;
	std::vector<float> radiusOfCylinders;
	pcl::PointXYZ minPoint;
	pcl::PointXYZ closestJoint;
	//float distance;
	std_msgs::Float32MultiArray returnArray;
	int numberOfClusters;

public:
	DistanceHandler(ros::NodeHandle& nh) : tvec(3,1,cv::DataType<float>::type) {
		calibrationSubscriber = nh.subscribe("calibration_data", 1, &DistanceHandler::calibrationCallback, this);
		//TODO add robotSubsriber
		clusteringSubscriber = nh.subscribe("cluster_vectors", 1,  &DistanceHandler::distanceCallback, this);
		distancePublisher = nh.advertise<std_msgs::Float32MultiArray>("distances", 1);
		cylinderPublisehr = nh.advertise<visualization_msgs::Marker>("robot_cylinder", 0);
		//only for testing
		cv::Mat test;
		test = (cv::Mat_<float>(3,1) <<1.00f,0.0f,0.0f);
		robotJoint.push_back(test);
		test =(cv::Mat_<float>(3,1) <<1.00f,0.0f,2.0f);
		robotJoint.push_back(test);
		sqrLengthBetweenJoints.push_back(1.0f);
		radiusOfCylinders.push_back(0.01f);
		insideRobotParameter = 0.4;

		cylinder.header.frame
	}
	void calibrationCallback(const std_msgs::Float64MultiArray& msg){
		cv::Mat rvec(3,1,cv::DataType<float>::type);
		std::cout << "Calibrating" << std::endl;
		for (int i = 0; i <3; ++i){
			rvec.at<float>(i,0) = msg.data[i];
			tvec.at<float>(i,0) = msg.data[i+3];
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
			cv::Mat jointInKinectCoord = rotationMatrix*robotJoint[i]+tvec;
			float distance = powf(jointInKinectCoord.at<float>(0,0)-x, 2)+powf(jointInKinectCoord.at<float>(1,0)-y, 2)+powf(jointInKinectCoord.at<float>(2,0)-z, 2);
			if (distance <= minDistance)
			{
				minDistance = distance;
				minPoint.x = x;
				minPoint.y = y;
				minPoint.z = z;
				closestJoint.x = robotJoint[i].at<float>(0,0);
				closestJoint.y = robotJoint[i].at<float>(1,0);
				closestJoint.z=robotJoint[i].at<float>(2,0);//should produce errors
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

			std::cout << "Object information for object " << j << std::endl;
			std::cout << "closest joint is " << closestJoint.x << " " << closestJoint.y << " " << closestJoint.z << std::endl;
			std::cout << "Distance to robot is " << minDistance << std::endl;

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
			std::cout << "***** rMatrixInfoSTART *****" << std::endl;
			std::cout << "Joint0: "<< rotationMatrix*robotJoint[0]+tvec << std::endl;
			std::cout << "Joint1: "<< rotationMatrix*robotJoint[1]+tvec << std::endl;
			std::cout << "***** rMatrixInfoEND *****" << std::endl;

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
			if (!rotationMatrix.empty()){
				cv::Mat joint1KinectCoord = rotationMatrix*robotJoint[i] + tvec;
				cv::Mat joint2KinectCoord = rotationMatrix*robotJoint[i+1] + tvec;

				if (pointInsideCylinder(joint1KinectCoord,joint2KinectCoord,sqrLengthBetweenJoints[i],radiusOfCylinders[i],p))
							return true;
			} // rotationMatrix*robotJoint[i] +tvec,rotationMatrix*robotJoint[i+1] +tvec,sqrLengthBetweenJoints[i],radiusOfCylinders[i],p
			else if (pointInsideCylinder(robotJoint[i],robotJoint[i+1],sqrLengthBetweenJoints[i],radiusOfCylinders[i],p)){
				std::cout << "Warning!, Calibration is not working" << std::endl;
				return true;
			}
		}
		return false;
	}

	//Algorithm copied from http://www.flipcode.com/archives/Fast_Point-In-Cylinder_Test.shtml
	bool pointInsideCylinder( cv::Mat& pt1, cv::Mat& pt2, float lengthsq, float radius_sq, const clustering::point& testpt )
	{
		float dx, dy, dz;	// vector d  from line segment point 1 to point 2
		float pdx, pdy, pdz;	// vector pd from point 1 to test point
		float dot, dsq;

		dx = pt2.at<float>(0,0) - pt1.at<float>(0,0);	// translate so pt1 is origin.  Make vector from
		dy = pt2.at<float>(1,0) - pt1.at<float>(1,0);     // pt1 to pt2.  Need for this is easily eliminated
		dz = pt2.at<float>(2,0) - pt1.at<float>(2,0);

		pdx = testpt.x - pt1.at<float>(0,0);		// vector from pt1 to test point.
		pdy = testpt.y - pt1.at<float>(1,0);
		pdz = testpt.z - pt1.at<float>(2,0);


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
