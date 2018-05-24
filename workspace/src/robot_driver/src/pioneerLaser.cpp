// Includes for multiple files
#include "includes.h"
#include "GridObject.hpp"
#include "ShapeList.hpp"

using namespace std;

geometry_msgs::Twist velocityCommand; 

bool isOnBorder(int currentX, int currentY);

// Globals (odometry)
Pose currentPose;
bool constantGradient;
ShapeList objectContainer;
int grid_x; 
int grid_y;
float map_o_x = 0;
float map_o_y = 0;
float map_r = 1;

bool turning = false;
int objectcounter = 0;
bool blockedRight, blockedLeft = false;

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
	objectcounter = 0;
	float sumLeft, sumRight = 0.0;
	int sumLeftCounter, sumRightCounter = 0;
	bool blocked = false;
	
	// Go through the laser data
	for(int j = 0; j < rangeDataNum - 1; ++j) { 
		// Determine where the blockages are
		
		if ((laserScanData->ranges[j] < 0.7) && !turning) {
			if (j < 128) {
				blockedRight = true;
				//cout << "Right blocking \n";
				sumRight += laserScanData->ranges[j];
				sumRightCounter++;
			} else if (j >= 384) {
				blockedLeft = true;
				//cout << "Left blocking \n";
				sumLeft += laserScanData->ranges[j];
				sumLeftCounter++;
			}
		}

		if (laserScanData->ranges[j] < 0.7) {
			if ((j >= 128) && (j < 384)) {
				blocked = true;
			}
		}
		
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
				} else {
					// Object does not have the right angles
					newObject->setDefaultShapeType();
				}
				// NEW SHAPE DETECTED, ADD TO VECTOR IF IT IS UNIQUE//
				objectContainer.addShape(newObject->getShapeType(), newObject->getWidth(), newObject->getLength(), newObject->getRadius(), newObject->getCurrentX(), newObject->getCurrentY());
				objectcounter++;
				delete newObject;
			}
			endSearchIndex = 0;
		}
	}
	
	//cout << "Number of objects detected now: " << objectcounter << "\n";
	// Process which way to move
	
	if (blocked) {
		// Something ahead
		velocityCommand.linear.x = 0;
		turning = true;
		if (blockedRight && !blockedLeft) {
			// turn left
			cout << "Must turn left\n";
			velocityCommand.angular.z = 1.5; // turn left
		} else if (!blockedRight && blockedLeft) {
			// turn right
			cout << "Must turn right\n";
			velocityCommand.angular.z = -1.5; // turn right
		} else {
			// Select the direction with the longest side to turn
			if ((sumRight/sumRightCounter) > (sumLeft/sumLeftCounter)) {
				// turn to the right
				cout << "Turning right cause longer " << (sumRight/sumRightCounter) << "\n";
				velocityCommand.angular.z = -1.5; // turn right
			} else if ((sumRight/sumRightCounter) < (sumLeft/sumLeftCounter)) {
				cout << "Turning left cause longer " << (sumLeft/sumLeftCounter) << "\n";
				velocityCommand.angular.z = 1.5; // turn left
			} else {
				cout << "too wide\n";
				if (laserScanData->ranges[0] > laserScanData->ranges[rangeDataNum-1]) {
					// turn right
					
					velocityCommand.angular.z = -1.5;
				} else {
					// turn left
					velocityCommand.angular.z = 1.5;
				}
			}
		}
	} else {
		// Not blocked so keep going straight
		turning = false;
		blockedRight = false;
		blockedLeft = false;
		velocityCommand.linear.x = 0.75;   // stop forward movement
		velocityCommand.angular.z = 0; // turn left
		
	}
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

/*
The OccupancyGrid is a 2-D grid map, in which 
each cell represents the probability of an object.
http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
*/
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) { 
	//get the origin of the map frame
	map_o_x = msg->info.origin.position.x;
	map_o_y = msg->info.origin.position.y;
	//get the resolution of each cell in the OccupancyGrid
	map_r = msg->info.resolution;
	
	// Go through the entire shape list - determine if the shape actually exists where it says it exists
	int listSize = objectContainer.getShapeListSize();
	
	//the occupancy grid is a 1D array representation of a 2-D grid map
	//compute the robot position in the 1D array
	int r_index = grid_x + grid_y * msg->info.width;
	//cout << r_index << "\n";
	int printEnable = 0;
	float shapeXLoc = 0.0;
	float shapeYLoc = 0.0;
	int pointIndex = 0;
	
	for(int i = 0; i < listSize; i++) {
		// Get the current XY and check if they exist
		shapeXLoc = objectContainer.getShapeCurrentX(i);
		shapeYLoc = objectContainer.getShapeCurrentY(i);
		
		// Convert to index
		pointIndex = round(shapeXLoc) + round(shapeYLoc)*msg->info.width;
		
		if (msg->data[pointIndex] == 0 || isOnBorder(shapeXLoc, shapeYLoc)) {
			// There is no shape there
			objectContainer.removeElement(i);
			cout << "Removing the element\n";
			return;
		}	
	}
	
	objectContainer.printShapes();
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
	
	/*
	subscribe to the map created by gmapping
	*/
	ros::Subscriber sub_map = my_handle.subscribe("map", 1, mapCallback);

	ros::Rate loop_rate(1000);// loop 10 Hz
	
	objectContainer = ShapeList();
	
	// publish the velocity set in the call back
	tf::TransformListener listener;
	
	int random_direction = (rand() % 10) + 20;
	velocityCommand.linear.x = 0;
	velocityCommand.angular.z = 1.5;
	
	for (int i = 0; i < random_direction; i++) {
		vel_pub_object.publish(velocityCommand);
		
		//ros::spinOnce();
		
		loop_rate.sleep();
		
		
		tf::StampedTransform transform;
		try{
			//transform the coordinate frame of the robot to that of the map
			//(x,y) index of the 2D Grid
			listener.lookupTransform("map", "base_link",ros::Time(0), transform);
			grid_x = (unsigned int)((transform.getOrigin().x() - map_o_x) / map_r);
			grid_y = (unsigned int)((transform.getOrigin().y() - map_o_y) / map_r);
			//cout << "Gridy: " << grid_y << "   GridX: " << grid_x << "\n";
		}
		catch (tf::TransformException ex){
			//ROS_ERROR("%s\n",ex.what());
			ros::Duration(1.0).sleep();
		}
	}
	
	while(ros::ok()) {
		
		vel_pub_object.publish(velocityCommand);
		
		ros::spinOnce();
		
		loop_rate.sleep();
		
		
		tf::StampedTransform transform;
		try{
			//transform the coordinate frame of the robot to that of the map
			//(x,y) index of the 2D Grid
			listener.lookupTransform("map", "base_link",ros::Time(0), transform);
			grid_x = (unsigned int)((transform.getOrigin().x() - map_o_x) / map_r);
			grid_y = (unsigned int)((transform.getOrigin().y() - map_o_y) / map_r);
		}
		catch (tf::TransformException ex){
			ros::Duration(1.0).sleep();
		}
	}

	return 0;
}


/* HELPER REGION */
bool isOnBorder(int currentX, int currentY) {
	if ((currentX <= 0) || (currentX >= 10) || (currentY >= 3) || (currentY <= -3)) {
		return true;
	}
	return false;
}


