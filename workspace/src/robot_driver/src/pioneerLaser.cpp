// Includes for multiple files
#include "includes.h"
#include "GridObject.hpp"
#include "ShapeList.hpp"

using namespace std;

geometry_msgs::Twist velocityCommand; 
void incrementPointer(int &pointer);
float cross(Coord A, Coord C);
float cosineRule(float distanceA, float distanceB, float angleC);
void determineLocation(float distanceA, float angleA, float distanceB, float angleB);
float determinePose(float distanceA, float angleA, float distanceB, float angleB, float distanceC, float angleC);
void getCoords(GridObject newObject);

// Globals (odometry)
Pose currentPose;
bool constantGradient;

/*
The scan subscriber call back function
To understand the sensor_msgs::LaserScan object look at
http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
*/
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData) {
	// Compute the number of data points
	// max angle and min angle of the laser scanner divide by the increment angle of each data point
	constantGradient = false;
	float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);

	detectionType currentType = noEdge;
	detectionType previousType = noEdge;
	bool detecting = false;
	int startSearchIndex = 0;
	int middleSearchIndex = 0;
	int endSearchIndex = 0;

	int detectedIndex = 0;
	
	// Current define the current status of the rangefinder unit:
	// 1 - increasing 
	// 2 - decreasing
	// 3 - gap
	int currentStatus = 0;
	int previousStatus = 0;
	
	// Go through the laser data
	for(int j = 0; j < rangeDataNum - 1; ++j) { 
		//**** Code to detect if increasing or decreasing gradient ****//
		previousStatus = currentStatus;
		if (laserScanData->ranges[j] < laserScanData->ranges[j+1]) {
			// Increasing vertex
			currentStatus = 1;
		} else if (laserScanData->ranges[j] > laserScanData->ranges[j+1]) {
			// Decreasing vertex
			currentStatus = 2;
		} else {
			// Equal due to resolution of the points measured
			currentStatus = currentStatus;
		}
	
		if (abs(laserScanData->ranges[j] - laserScanData->ranges[j+1]) > CLEARANCE) {
			// Falling edge detected
			currentStatus = 3;
			previousType = currentType;
			currentType = ((laserScanData->ranges[j] - laserScanData->ranges[j+1]) > CLEARANCE) ? fallingEdge : risingEdge;
		} 

		if (currentStatus != previousStatus) {
			if (detecting && (currentStatus == 1)) {
				// Point of infection
				// j-1 -> j will provide decreasing measurement, j -> j+1 will provide increasing measurement
				// Angle in between will be used to determine if circle or square
				middleSearchIndex = j;
			}
			
			if ((currentStatus == 3)) {
				// Object detected
				// Eventually this will need to be placed into a linked list for defined searching
				// To store is the diatance and current odometry readings
				if ((previousType == fallingEdge) && (currentType == risingEdge)) {
					detecting = !detecting;
					endSearchIndex = j;
				} else if (currentType == fallingEdge) {
					detecting = !detecting;
					startSearchIndex = j;
				}
			}
		}
		//********************************************************//
		
		// Object has been found - perform test for type of object, size and location:
		if (endSearchIndex != 0) {
			if (((middleSearchIndex - startSearchIndex) > RANGE*FACTOR)  && ((endSearchIndex - middleSearchIndex) > RANGE*FACTOR)) {
				// Location to place the new grid object
				GridObject * newObject = new GridObject(laserScanData->ranges[startSearchIndex+1], laserScanData->ranges[middleSearchIndex], laserScanData->ranges[endSearchIndex], startSearchIndex+1, middleSearchIndex, endSearchIndex);
				float angle = newObject->determinePose(laserScanData->ranges[middleSearchIndex-RANGE], (-1.5708 + (middleSearchIndex-RANGE)*laserScanData->angle_increment), laserScanData->ranges[middleSearchIndex], (-1.5708 + middleSearchIndex*laserScanData->angle_increment), laserScanData->ranges[middleSearchIndex+RANGE], (-1.5708 + (middleSearchIndex+RANGE)*laserScanData->angle_increment));

				if (angle < ANGLE_90) {
					// Check if the lengths are uneven using cosine rule - set a tolerance
					newObject->determineShapeSR(laserScanData->ranges[startSearchIndex+1], (1.5708-(startSearchIndex+1)*laserScanData->angle_increment), laserScanData->ranges[endSearchIndex], (1.5708-(endSearchIndex)*laserScanData->angle_increment), laserScanData->angle_increment);
					newObject->determineGlobalLocation(currentPose.poseX, currentPose.poseY, -currentPose.yaw);
				} else if (angle > ANGLE_180) {
					// Determine the gradient changes
					int index = 0;
					if ((middleSearchIndex - startSearchIndex) > (endSearchIndex - middleSearchIndex)) {
						// Search between start and middle
						index = (middleSearchIndex - startSearchIndex)/2 + startSearchIndex;
					} else {
						// Search between middle and end
						index = (endSearchIndex - middleSearchIndex)/2 + middleSearchIndex;
					}
				
					newObject->determineShapeC(laserScanData->ranges[index-RANGE], laserScanData->ranges[index], laserScanData->ranges[index+RANGE], laserScanData->angle_increment, index);
					newObject->determineGlobalLocation(currentPose.poseX, currentPose.poseY, -currentPose.yaw);
				}
				// NEW SHAPE DETECTED, ADD TO VECTOR IF IT IS UNIQUE//
				
				
				delete newObject;
			}
			endSearchIndex = 0;
		}
	}

	
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) { 
	//get the origin of the map frame
	
	
	//the occupancy grid is a 1D array representation of a 2-D grid map
	//compute the robot position in the 1D array
	
}

/*
The odometer call back function
http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
*/
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	// Note: is that the position acquisition and lidar acquisition is not synchronised
	currentPose.poseX = msg->pose.pose.position.x;
	currentPose.poseY = msg->pose.pose.position.y;

	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
		
	double roll, pitch, yaw; 
	m.getRPY(roll, pitch, yaw);
	currentPose.roll = roll;
	currentPose.pitch = pitch;
	currentPose.yaw = yaw;
}

int main (int argc, char **argv) {	
	ros::init(argc, argv, "pioneer_laser_node");	// command line ROS arguments
	ros::NodeHandle my_handle;	// ROS comms access point

	ros::Publisher vel_pub_object = my_handle.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);
	ros::Subscriber sub = my_handle.subscribe("/odom", 1000, chatterCallback);
	
	/*
	subscribe to the scan topic and define a callback function to process the data
	the call back function is called each time a new data is received from the topic
	*/
	ros::Subscriber laser_sub_object = my_handle.subscribe("/scan", 1, laserScanCallback);
	
	ros::Subscriber sub_map = my_handle.subscribe("map", 1, mapCallback);

	ros::Rate loop_rate(10);// loop 10 Hz
	
	
	tf::TransformListener listener;
	// publish the velocity set in the call back
	while(ros::ok()) {
		ros::spinOnce();
		// Do some calculation 
		
		loop_rate.sleep();
		
		tf::StampedTransform transform;
		try{
			//transform the coordinate frame of the robot to that of the map
			//(x,y) index of the 2D Grid
			listener.lookupTransform("map", "base_link",ros::Time(0), transform);
			
			cout << "Global X: " << transform.getOrigin().x() << "\n";
			cout << "Global Y: " << transform.getOrigin().y() << "\n";
			cout << "Rotation: " << currentPose.roll << " : " << currentPose.pitch << " : " << currentPose.yaw << "\n";
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s\n",ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	return 0;
}

// HELPER REGION //

float cross(Coord A, Coord C) {
	float dot = A.x*C.x + A.y*C.y;
	float det = A.y*C.x - A.x*C.y;
	return atan2(det, dot);
}

float cosineRule(float distanceA, float distanceB, float angleC) { 
	return sqrt(pow(distanceA, 2.0) + pow(distanceB, 2.0) - 2*distanceA*distanceB*cos(angleC));
}

void getCoords(GridObject newObject) {
	geometry_msgs::PointStamped globalObjectCoords;
	globalObjectCoords.header.frame_id = "base_link";
	
	globalObjectCoords.header.stamp = ros::Time();
	
	globalObjectCoords.point.x = newObject.getCurrentX();
	globalObjectCoords.point.y = newObject.getCurrentY();
	globalObjectCoords.point.z = 0.0;
	
	tf::Transformer transformer(true, ros::Duration(10));
}




