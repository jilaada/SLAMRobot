// Includes for multiple files
#include "includes.h"
#include "edge.hpp"

using namespace std;

geometry_msgs::Twist velocityCommand; 
void incrementPointer(int &pointer);
float cross(Coord A, Coord C);
float cosineRule(float distanceA, float distanceB, float angleC);
void determineLocation(float distanceA, float angleA, float distanceB, float angleB);
float determinePose(float distanceA, float angleA, float distanceB, float angleB, float distanceC, float angleC);

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
	}

	// Object has been found - perform test for type of object, size and location:
	if (endSearchIndex != 0) {
		if (((middleSearchIndex - startSearchIndex) > RANGE*FACTOR)  && ((endSearchIndex - middleSearchIndex) > RANGE*FACTOR)) {
			float angle = determinePose(laserScanData->ranges[middleSearchIndex-RANGE], (-1.5708 + (middleSearchIndex-RANGE)*laserScanData->angle_increment), laserScanData->ranges[middleSearchIndex], (-1.5708 + middleSearchIndex*laserScanData->angle_increment), laserScanData->ranges[middleSearchIndex+RANGE], (-1.5708 + (middleSearchIndex+RANGE)*laserScanData->angle_increment));
			float x = 0;
			float y = 0;
			if (angle < ANGLE_90) {
				// Check if the lengths are uneven using cosine rule - set a tolerance
				
				x = cosineRule(laserScanData->ranges[startSearchIndex+1], laserScanData->ranges[middleSearchIndex], (middleSearchIndex-startSearchIndex-1)*laserScanData->angle_increment);
				y = cosineRule(laserScanData->ranges[endSearchIndex], laserScanData->ranges[middleSearchIndex], (endSearchIndex-middleSearchIndex)*laserScanData->angle_increment);
				
				if ((x > y-TOLERANCE) && (x < y+TOLERANCE)) {
					// Equal lengths - this is square
					cout << "Found a square!    " << "x: " << x << " : " << "y: " << y << "\n";
					determineLocation(laserScanData->ranges[startSearchIndex+1], (1.5708-(startSearchIndex+1)*laserScanData->angle_increment), laserScanData->ranges[endSearchIndex], (1.5708-(endSearchIndex)*laserScanData->angle_increment));
				} else {
					cout << "Found a rectangle! " << "x: " << x << " : " << "y: " << y << "\n";
					determineLocation(laserScanData->ranges[startSearchIndex+1], (1.5708-(startSearchIndex+1)*laserScanData->angle_increment), laserScanData->ranges[endSearchIndex], (1.5708-(endSearchIndex)*laserScanData->angle_increment));
				}
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
				// Determine the constantGradient parameter
				float ignoreValue = determinePose(laserScanData->ranges[index-RANGE], (-1.5708 + (index-RANGE)*laserScanData->angle_increment), laserScanData->ranges[index], (-1.5708 + index*laserScanData->angle_increment), laserScanData->ranges[index+RANGE], (-1.5708 + (index+RANGE)*laserScanData->angle_increment));
			
				if (constantGradient) {
					// Potential for vertex of square OR rectangle therefore unsure
					cout << "More Info Required!\n";
				} else {
					cout << "Found a circle     " << "Radius: " << cosineRule(laserScanData->ranges[startSearchIndex+1], laserScanData->ranges[endSearchIndex], (endSearchIndex-startSearchIndex-1)*laserScanData->angle_increment)/2 << "\n";
					determineLocation(laserScanData->ranges[startSearchIndex+1], (1.5708-(startSearchIndex+1)*laserScanData->angle_increment), laserScanData->ranges[endSearchIndex], (1.5708-(endSearchIndex)*laserScanData->angle_increment));
				}
			}
		}
		
		
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

	ros::Rate loop_rate(10);// loop 10 Hz

	// publish the velocity set in the call back
	while(ros::ok()) {
		ros::spinOnce();
		// Do some calculation 
		
		loop_rate.sleep();
	}

	return 0;
}

// HELPER REGION //

void incrementPointer(int &pointer) {
	if (pointer == (BUFFER_SIZE - 1)) {
		pointer = 0;
	} else {
		pointer++;
	}
}

float determinePose(float distanceA, float angleA, float distanceB, float angleB, float distanceC, float angleC) {
	// Current position is a global so should be ok
	float xA = distanceA * cos(angleA);
	float yA = abs(distanceA * sin(angleA));
	float xB = distanceB * cos(angleB);
	float yB = abs(distanceB * sin(angleB));
	float xC = distanceC * cos(angleC);
	float yC = abs(distanceC * sin(angleC));
	
	Coord A;
	Coord C;
	
	A.x = xA - xB;
	A.y = -(abs(yA-yB));
	C.x = xC - xB;
	C.y = abs(yC-yB);
	
	if (((yA-yB)/abs(xA-xB) < ((yB-yC)/abs(xB-xC))+TOLERANCE) && ((yA-yB)/abs(xA-xB) > ((yB-yC)/abs(xB-xC))-TOLERANCE)) {
		// Constant gradient detected
		constantGradient = true;
	} else {
		constantGradient = false;
	}

	return abs(cross(A,C));
}

void determineLocation(float distanceA, float angleA, float distanceB, float angleB) {
	float xA = distanceA * cos(angleA);
	float yA = distanceA * sin(angleA);
	float xB = distanceB * cos(angleB);
	float yB = distanceB * sin(angleB);
	cout << "distanceA: " << distanceA << "\n";
	cout << "angleA: " << angleA << "\n";
	cout << "distanceB: " << distanceB << "\n";
	cout << "angleB: " << angleB << "\n";
	
	float midX;
	
	if (xA > xB) {
		midX = (xA-xB)/2 + xB + LASER_POSE;
	} else if (xB > xA) {
		midX = (xB-xA)/2 + xA + LASER_POSE;
	}
	
	float midY = (yB - yA)/2 + yA;
	
	//midX = midX + LASER_POSE + MIN(xA, xB);
	
	cout << "Location x: " << midX << "\n";
	cout << "Location y: " << midY << "\n";
}

float cross(Coord A, Coord C) {
	float dot = A.x*C.x + A.y*C.y;
	float det = A.y*C.x - A.x*C.y;
	return atan2(det, dot);
}

float cosineRule(float distanceA, float distanceB, float angleC) { 
	return sqrt(pow(distanceA, 2.0) + pow(distanceB, 2.0) - 2*distanceA*distanceB*cos(angleC));
}




