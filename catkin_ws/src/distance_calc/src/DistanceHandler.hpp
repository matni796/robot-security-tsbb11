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
#include <tf/transform_listener.h>



struct ObjectData{
	float minDistance;
	pcl::PointXYZ minPoint;
	int closestJoint;
	int inside;
	int outside;
	pcl::PointXYZ meanPoint;
	bool visible;
	clustering::pointArray cloud;
}; //index

struct SecurityDistances{
	float redDistance;
	float yellowDistance;
	float greenDistance;
	float trackingDistance;
	float trackingMin;
};

struct ObjectDataList{
	std::vector<ObjectData> list;
	int closestObject; //index
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
	bool tracking;
	std::vector<clustering::point> robotJoint;
	std::vector<float> lengthBetweenJoints;
	std::vector<float> radiusOfCylinders;
	float radiusOfCylinderParameter;
	std::string jointNames[7] ;
	pcl::PointXYZ minPoint;
	int numberOfClusters;
	pcl::PointCloud<pcl::PointXYZ>::Ptr publishedPointCloud;
	SecurityDistances safetyZones;	
	tf::TransformListener listener;	

public:
	DistanceHandler(ros::NodeHandle& nh) :
		publishedPointCloud(new pcl::PointCloud<pcl::PointXYZ>())   {
		//calibrationSubscriber = nh.subscribe("calibration_data", 1, &DistanceHandler::calibrationCallback, this);
		//TODO add robotSubsriber
		clusteringSubscriber = nh.subscribe("cluster_vectors", 1,  &DistanceHandler::distanceCallback, this);
		linePublisher = nh.advertise<visualization_msgs::Marker>("closest_line", 0);
		pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("objects",0);
		objects.closestObject = -1;
		
		//only for testing
		/*		cv::Mat test;
		test = (cv::Mat_<float>(3,1) <<-1.0f,0.0f,3.5f);
		robotJoint.push_back(test);
		test =(cv::Mat_<float>(3,1) <<-1.0f,0.0f,3.5f);
		robotJoint.push_back(test);
		lengthBetweenJoints.push_back(1.0f);*/
		
		//Some parameters
		insideRobotParameter = 0.3;
		radiusOfCylinderParameter = 0.09f;
		tracking = false;

		//initializing the closest line.
		line.header.frame_id = "/camera_depth_optical_frame";
		line.header.stamp=ros::Time();
		line.id = 0;
		line.type = visualization_msgs::Marker::LINE_STRIP;
		line.action = visualization_msgs::Marker::ADD;

		//initialize security distances should be sqrtdded
		safetyZones.redDistance = 0.5f;
		safetyZones.yellowDistance = 0.8f;
		safetyZones.greenDistance = 1.2f;
		safetyZones.trackingDistance = 1.1f;
		safetyZones.trackingMin = 1.0f;

		//defining jointNames
		jointNames[0]="/link_s";
		jointNames[1]="/link_l";
		jointNames[2]="/link_e";
		jointNames[3]="/link_u";
		jointNames[4]="/link_r";
		jointNames[5]="/link_b";
		jointNames[6]="/link_t";
	}

	///Functions regarding calculation of distance
	void distanceCallback(clustering::clusterArray msg){ //This function is what's doing all the work.
		rawClusters = msg;
		if(!updateRobotCoordinates())
			return;
		if(tracking)
			newCloudsHandler(msg);
		else
			removeRobot(msg);
		setClosestObject();
		publishLine();
		publishPointCloud(objects.clouds);
		displayCurrentStatus();
	}

	void publishPointCloud(clustering::clusterArray& clusters){
		std::vector<pcl::PointXYZ> tempObjectVector;
		publishedPointCloud->clear();
		publishedPointCloud->header.frame_id= "/camera_depth_optical_frame";
		ROS_INFO("Creating object point cloud from %d clusters\n", clusters.ca.size());
		for(int i=0; i<clusters.ca.size();++i){
			for (int k= 0; k<clusters.ca[i].pa.size();++k){
				tempObjectVector.push_back(pcl::PointXYZ(clusters.ca[i].pa[k].x,clusters.ca[i].pa[k].y,clusters.ca[i].pa[k].z));
			}
		}
		publishedPointCloud->insert(publishedPointCloud->begin(),tempObjectVector.begin(),tempObjectVector.end());
		ROS_INFO("Publishing point cloud of size %dx%d to objects", publishedPointCloud->width, publishedPointCloud->height);
		pointCloudPublisher.publish(publishedPointCloud);
	}

	void publishLine(){
		if(objects.closestObject!= -1){
			clustering::point joint = robotJoint[objects.list[objects.closestObject].closestJoint];
			pcl::PointXYZ objectPoint = objects.list[objects.closestObject].minPoint;
			float distance = objects.list[objects.closestObject].minDistance;			
			if (distance <=safetyZones.greenDistance){
				geometry_msgs::Point p;
				p.x=joint.x;
				p.y=joint.y;
				p.z= joint.z;
				line.points.push_back(p);
				p.x=objectPoint.x;
				p.y=objectPoint.y;
				p.z=objectPoint.z;
				line.points.push_back(p);
				line.scale.x=0.05;				
				if (distance <= safetyZones.redDistance){
					line.color.a=1.0;
					line.color.g=0.0;
					line.color.b=0.0;
					line.color.r=1.0;
				} else if (distance <= safetyZones.yellowDistance){
					line.color.a=1.0;
					line.color.g=1.0;
					line.color.b=0.0;
					line.color.r=1.0;
				} else{
					line.color.a=1.0;
					line.color.g=1.0;
					line.color.b=0.0;
					line.color.r=0.0;
				}
			} 
			linePublisher.publish(line);
			line.points.clear();
		}
	}


	float compareToRobot(float x, float y, float z, cv::Mat& joint){
		float distance = sqrt(powf(joint.at<float>(0,0)-x, 2)+powf(joint.at<float>(1,0)-y, 2)+powf(joint.at<float>(2,0)-z, 2));
		return distance;
	}

	// This is a big function.
	// Process new point cloud. Make matches with previous and checks what belongs to the robot.
	// Also calculates the distance between each point cloud, that doesn't belong to robot, and
	// closest joint.
	//It removes the clouds that belongs to the robot.
	void newCloudsHandler(clustering::clusterArray& rawClusterCloud) {
		objects.clouds.ca.clear();
		// Sets every object/cloud from previous frame to invisible.
		for (int i = 0; i < objects.list.size(); i++) {
			objects.list[i].visible = false;
		}
		//objects.list.clear();

		objects.closestObject = -1;
		robot.list.clear();

		for(int i=0; i<rawClusterCloud.ca.size(); i++) {
			ObjectData data;
			data.cloud.pa = rawClusterCloud.ca[i].pa;
			data.visible = true;
			data.minDistance = 1000.0f;
			data.closestJoint = -1;
			data.outside = 0;
			data.inside = 0;
			++numberOfClusters;
			for(int j=0; j<rawClusterCloud.ca[i].pa.size(); j++) {
				pointInsideRobot(rawClusterCloud.ca[i].pa[j], data);
				data.meanPoint.x += rawClusterCloud.ca[i].pa[j].x;
				data.meanPoint.y += rawClusterCloud.ca[i].pa[j].y;
				data.meanPoint.z += rawClusterCloud.ca[i].pa[j].z;
			}
			// Calculate the mean point for each cluster
			data.meanPoint.x = data.meanPoint.x / rawClusterCloud.ca[i].pa.size();
			data.meanPoint.y = data.meanPoint.y / rawClusterCloud.ca[i].pa.size();
			data.meanPoint.z = data.meanPoint.z / rawClusterCloud.ca[i].pa.size();
			// Finding the closest matching cloud from previous frame(similar mean value)
			float cloudDistance;
			int matchingCloud;
			float nearestCloud = safetyZones.trackingMin;
			bool cloudFound = false;
			for (int k = 0; k < objects.list.size(); k++)
				if(!objects.list[i].visible){{
						cloudDistance = sqrt(powf(objects.list[k].meanPoint.x - data.meanPoint.x, 2) +
											 powf(objects.list[k].meanPoint.y - data.meanPoint.y, 2) +
											 powf(objects.list[k].meanPoint.z - data.meanPoint.z, 2));
						if (cloudDistance < nearestCloud){

							matchingCloud = k;
							nearestCloud = cloudDistance;
							cloudFound = true;
						}
					}
				}
			// Update list of object
			if(cloudFound){
				objects.list[matchingCloud] = data;

				std::cout << "Cloud is tracked! \n" << std::endl;
			}

			// if statement checks if the object belongs to the robot or not
			if((data.inside+data.outside != 0)&&!(((float)data.inside)/((float)data.inside+(float)data.outside) > insideRobotParameter)){
				//objects.clouds.ca.push_back(rawClusterCloud.ca[i]);
				if(!cloudFound && data.minDistance > safetyZones.trackingDistance){
					objects.list.push_back(data);
					std::cout << "New cloud! \n" << std::endl;
				}
			} else{// This is a robot object
				robot.clouds.ca.push_back(rawClusterCloud.ca[i]);
				robot.list.push_back(data);
			}

		}

		// Check if any of the invisible objects could have disappeared out of the frame.
		for (int i = 0; i < objects.list.size(); i++){
			if (objects.list[i].visible == false && objects.list[i].minDistance > safetyZones.trackingDistance){
				std::cout << "Remove object from list" << std::endl;
				objects.list.erase (objects.list.begin() + i);// Erases the object list at position i.								
			}
		}
		for(int i = 0; i < objects.list.size(); i++){
			objects.clouds.ca.push_back(objects.list[i].cloud);
		}
	}

	//Functions regarding removal of the robot without tracking!
	void removeRobot(clustering::clusterArray& rawClusterCloud){
		objects.clouds.ca.clear();
		objects.list.clear();
		objects.closestObject = -1;
		robot.list.clear();

		for(int i=0; i<rawClusterCloud.ca.size(); i++) {
			ObjectData data;
			data.minDistance = 1000.0f;
			data.closestJoint = -1;
			data.inside = 0;
			data.outside= 0;
			++numberOfClusters;
			for(int j=0; j<rawClusterCloud.ca[i].pa.size(); j++)
				pointInsideRobot(rawClusterCloud.ca[i].pa[j], data);

			int total = data.inside+data.outside;
			if(total != 0 && (float)data.inside / total < insideRobotParameter) {
				objects.clouds.ca.push_back(rawClusterCloud.ca[i]);
				objects.list.push_back(data);
			} else{
				robot.clouds.ca.push_back(rawClusterCloud.ca[i]);
				robot.list.push_back(data);
			}
			std::cout << "outside: " << data.outside << std::endl;
			std::cout << "inside: " << data.inside << std::endl;
		}
	}

	void pointInsideRobot(clustering::point inputPoint, ObjectData& data){
		calculateClosestJoint(inputPoint, data);

		for(int i = 0; i < robotJoint.size()-1; ++i) {
			float radius = radiusOfCylinders[i], length = lengthBetweenJoints[i];
			bool res = pointInsideCylinder(robotJoint[i],
										   robotJoint[i+1],
										   length*length,
										   radius*radius,
										   inputPoint,
										   data);
			if(res) {
				++data.inside;
				return;
			}
		}
		++data.outside;
	}
	
	void calculateClosestJoint(clustering::point testpt, ObjectData & data) {
		for(int i = 0; i < robotJoint.size(); ++i) {
			float dx = testpt.x - robotJoint[i].x;
			float dy = testpt.y - robotJoint[i].y;
			float dz = testpt.z - robotJoint[i].z;
			float dist = sqrt(dx*dx+dy*dy+dz*dz);
			if(dist < data.minDistance) {
				data.minDistance = dist;
				data.minPoint.x = testpt.x;
				data.minPoint.y = testpt.y;
				data.minPoint.z = testpt.z;
				data.closestJoint = i;
			}
		}
	}

	//Algorithm copied from http://www.flipcode.com/archives/Fast_Point-In-Cylinder_Test.shtml
	bool pointInsideCylinder( clustering::point pt1, clustering::point pt2, float lengthsq, float radius_sq,
							  const clustering::point& testpt, ObjectData& data)
	{
		if(pt1.x == pt2.x && pt1.y == pt2.y && pt1.z == pt2.z)
			return false;
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
		if(dot < 0.0f || dot > lengthsq)
			return false; // outside cylinder top/bottom bounds
		dsq = (pdx*pdx + pdy*pdy + pdz*pdz) - dot*dot/lengthsq;
		if(dsq > radius_sq)
			return false; // outside cylinder radial bound

		return true; // inside cylinder

	}

	void setClosestObject(){
		float minDistance = 20000.0f;
		for (int i = 0;i<objects.list.size();++i){
			if (objects.list[i].minDistance < minDistance){
				minDistance = objects.list[i].minDistance;
				objects.closestObject= i;
				std::cout << objects.list[i].minDistance << "\n";
			}
		}
		std::cout <<"Size in setClosestObject(): " << objects.list.size() << " closest Object is: " << objects.closestObject << " closest dist: " << minDistance << std::endl;
	}

	bool updateRobotCoordinates(){
		robotJoint.clear();
		lengthBetweenJoints.clear();
		radiusOfCylinders.clear();
		tf::StampedTransform transform;
		cv::Mat oldJoint, joint;

		try {
			for(int i=0;i<7;++i){
				listener.lookupTransform("/camera_depth_optical_frame", jointNames[i],
										 ros::Time(0), transform);	
				tf::Vector3 origin = transform.getOrigin();
				float x = origin.x();
				float y = origin.y();
				float z = origin.z();

				clustering::point pt;
				pt.x = x; pt.y = y; pt.z = z;
				//printf("Robot joint coordinate: %.3f %.3f %.3f\n", x, y, z);
				joint = (cv::Mat_<float>(3,1) << x,y,z);
				robotJoint.push_back(pt);
				if(i > 0){
					double length = cv::norm(joint-oldJoint); // TODO: Check type
					lengthBetweenJoints.push_back(length);
					radiusOfCylinders.push_back(radiusOfCylinderParameter);
				}
				joint.copyTo(oldJoint);
				//dJoint = joint;
			}
			ROS_INFO("Succesful robot position update");
			return true;
		}
		catch(tf::TransformException ex) {
			ROS_INFO("Failed with getting transform: %s", ex.what());
			return false;	
		}
				
	}

	void displayCurrentStatus(){
		std::cout << "There are " << objects.list.size() << " objects visible\n";
		std::cout << "There are " << robot.list.size() << " clouds considered robot\n";
		if (objects.list.size() != 0){
			std::cout << "The closest object is " << objects.list[objects.closestObject].minDistance << " m away from joint number ";
			std::cout << "The closest object is " << objects.closestObject << "\n";
			for(int i=0; i<objects.list.size(); i++){
				std::cout << "object: "<< i<< " distance:" << objects.list[i].minDistance << "\n"; 
					
			}
		}
	}
};


#endif
