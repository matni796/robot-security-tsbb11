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



struct ObjectData{
	float minDistance =10000.0f;
	pcl::PointXYZ minPoint;
	int closestJoint=-1;
	int inside;
	int outside;
}; //index

struct ObjectDataList{
	std::vector<ObjectData> list;
	int closestObject=-1; //index
	clustering::clusterArray clouds;
};

class DistanceHandler{
private:
	ros::Subscriber calibrationSubscriber, robotSubscriber, clusteringSubscriber;
	ros::Publisher linePublisher, pointCloudPublisher;
	clustering::clusterArray rawClusters;
	float insideRobotParameter;
	visualization_msgs::Marker line;
	ObjectDataList objects;
	ObjectDataList robot;
	std::vector<cv::Mat_ <float> > robotJoint;
	std::vector<float> sqrLengthBetweenJoints;
	std::vector<float> radiusOfCylinders;
	pcl::PointXYZ minPoint;
	int numberOfClusters;
	pcl::PointCloud<pcl::PointXYZ>::Ptr publishedPointCloud;

public:
	DistanceHandler(ros::NodeHandle& nh) :
		publishedPointCloud(new pcl::PointCloud<pcl::PointXYZ>())   {
		//calibrationSubscriber = nh.subscribe("calibration_data", 1, &DistanceHandler::calibrationCallback, this);
		//TODO add robotSubsriber
		clusteringSubscriber = nh.subscribe("cluster_vectors", 1,  &DistanceHandler::distanceCallback, this);
		linePublisher = nh.advertise<visualization_msgs::Marker>("closest_line", 0);
		pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("objects",1);

		//only for testing
		cv::Mat test;
		test = (cv::Mat_<float>(3,1) <<1.00f,0.0f,100.0f);
		robotJoint.push_back(test);
		test =(cv::Mat_<float>(3,1) <<1.00f,0.0f,1.5f);
		robotJoint.push_back(test);
		test =(cv::Mat_<float>(3,1) <<-1.00f,0.0f,1.5f);
		robotJoint.push_back(test);
		sqrLengthBetweenJoints.push_back(1.0f);
		radiusOfCylinders.push_back(0.01f);
		insideRobotParameter = 0.4;

		line.header.frame_id = "/camera_depth_frame";
		line.header.stamp=ros::Time();
		line.id = 0;
		line.type = visualization_msgs::Marker::ARROW;
		line.action = visualization_msgs::Marker::ADD;

	}

	///Functions regarding calculation of distance
	void distanceCallback(clustering::clusterArray msg){ //This function is what's doing all the work.
		rawClusters = msg;
		removeRobot(msg);
		setClosestObject();
		//publishLine();
		publishPointCloud(objects.clouds);
		displayCurrentStatus();
	}

	void publishPointCloud(clustering::clusterArray& clusters){
		std::vector<pcl::PointXYZ> tempObjectVector;
		publishedPointCloud->clear();
		publishedPointCloud->header.frame_id= "/camera_depth_frame";
		for(int i=0; i<clusters.ca.size();++i){
			for (int k= 0; k<clusters.ca[i].pa.size();++k){
				tempObjectVector.push_back(pcl::PointXYZ(clusters.ca[i].pa[k].x,clusters.ca[i].pa[k].y,clusters.ca[i].pa[k].z));
			}
		}
		publishedPointCloud->insert(publishedPointCloud->begin(),tempObjectVector.begin(),tempObjectVector.end());
		pointCloudPublisher.publish(publishedPointCloud);
	}

	void publishLine(){
		if(objects.closestObject!= -1){
			cv::Mat joint = robotJoint[objects.list[objects.closestObject].closestJoint];
			pcl::PointXYZ objectPoint = objects.list[objects.closestObject].minPoint;
			/*line.pose.position.x=joint.at<float>(0,0);
			line.pose.position.y=joint.at<float>(1,0);
			line.pose.position.z=joint.at<float>(2,0);
			float diffX = joint.at<float>(0,0)- objectPoint.x;
			float diffY = joint.at<float>(1,0)- objectPoint.y;
			float diffZ = joint.at<float>(2,0)- objectPoint.z;
			line.pose.orientation.x =diffX;
			line.pose.orientation.y =diffY;
			line.pose.orientation.z =diffZ;*/
			line.points[0].x = joint.at<float>(0,0);
			line.points[0].y = joint.at<float>(1,0);
			line.points[0].z = joint.at<float>(2,0);
			line.points[1].x = objectPoint.x;
			line.points[1].y = objectPoint.y;
			line.points[1].z = objectPoint.z;
			//cylinder.pose.orientation.w = 10.0f;
			line.scale.x=0.1;
			line.scale.y=0.1;
			line.scale.z=0.1;
			line.color.a=1.0;
			line.color.g=1.0;
			line.color.b=0.0;
			line.color.r=0.0;
			linePublisher.publish(line);
		}
	}


	float compareToRobot(float x, float y, float z, cv::Mat& joint){
		float distance = powf(joint.at<float>(0,0)-x, 2)+powf(joint.at<float>(1,0)-y, 2)+powf(joint.at<float>(2,0)-z, 2);
		return distance;
	}

	//Functions regarding removal of the robot!
	void removeRobot(clustering::clusterArray& rawClusterCloud){
		objects.list.clear();
		objects.closestObject = -1;
		robot.list.clear();

		for(int i=0; i<rawClusterCloud.ca.size(); i++)
		{
			ObjectData data;
			++numberOfClusters;
			for(int j=0; j<rawClusterCloud.ca[i].pa.size(); j++)
			{
				pointInsideRobot(rawClusterCloud.ca[i].pa[j], data);
			}
			if((data.inside+data.outside != 0)&&!(data.inside/(data.inside+data.outside) > insideRobotParameter) ){
				objects.clouds.ca.push_back(rawClusterCloud.ca[i]);
				objects.list.push_back(data);
			} else{
				robot.clouds.ca.push_back(rawClusterCloud.ca[i]);
				robot.list.push_back(data);
			}
		}
	}

	void pointInsideRobot(clustering::point inputPoint, ObjectData& data){
		for(int i=0; i<robotJoint.size()-1; i++){
			pointInsideCylinder(robotJoint[i],robotJoint[i+1],sqrLengthBetweenJoints[i],radiusOfCylinders[i],inputPoint,data, i);
		}
	}

	//Algorithm copied from http://www.flipcode.com/archives/Fast_Point-In-Cylinder_Test.shtml
	void pointInsideCylinder( cv::Mat& pt1, cv::Mat& pt2, float lengthsq, float radius_sq,
			const clustering::point& testpt, ObjectData& data, int jointIndex)
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

		float dist = compareToRobot(testpt.x, testpt.y, testpt.z, pt2); //Will only calculate distance to the six first joints
		if (dist < data.minDistance)
		{
			data.minDistance = dist;
			data.minPoint.x = testpt.x;
			data.minPoint.y = testpt.y;
			data.minPoint.z = testpt.z;
			data.closestJoint= jointIndex;
		}

		if( dot < 0.0f || dot > lengthsq )
		{
			++data.outside;
		}
		else
		{
			dsq = (pdx*pdx + pdy*pdy + pdz*pdz) - dot*dot/lengthsq;

			if( dsq > radius_sq )
			{
				++data.inside;
			}
			else
			{
				++data.outside;		// return true if inside cylinder
			}
		}
	}

	void setClosestObject(){
		int minDistance = 20000.0f;
		for (int i = 0;i<objects.list.size();++i){
			if (objects.list[i].minDistance < minDistance){
				minDistance = objects.list[i].minDistance;
				objects.closestObject= i;
			}
		}
	}

	void displayCurrentStatus(){
		std::cout << "There are " << objects.list.size() << " objects visible\n";
		std::cout << "There are " << robot.list.size() << " clouds considered robot\n";
		if (objects.list.size() != 0){
			std::cout << "The closest object is " << objects.list[objects.closestObject].minDistance << " m away from joint number ";
			std::cout  << objects.list[objects.closestObject].closestJoint << "\n";
		}
	}
};


#endif
