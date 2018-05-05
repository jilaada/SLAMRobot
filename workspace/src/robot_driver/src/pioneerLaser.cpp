// Includes for multiple files
#include "includes.h"
#include "edge.hpp"

using namespace std;

geometry_msgs::Twist velocityCommand; 
void incrementPointer(int &pointer);
float cross(Coord A, Coord C);
float cosineRule(float distanceA, float distanceB, float angleC);
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
	// Rotating buffer array
	int bufferStatus[BUFFER_SIZE] = { };
	int bufferPointer = 0;
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
					cout << "OBJECT DETECTED\n";
					endSearchIndex = j;
				} else if (currentType == fallingEdge) {
					detecting = !detecting;
					cout << "POTENTIAL OBJECT DETECTED : " << detecting << "\n";
					startSearchIndex = j;
				}
			}
			//cout << currentStatus << " : " << laserScanData->ranges[j] << "\n"; 
		}
		// Buffer for noise: to be used later when I actually care about it
		bufferStatus[bufferPointer] = currentStatus;
		incrementPointer(bufferPointer);
	}

	// Object has been found - perform test for type of object, size and location:
	if (endSearchIndex != 0) {
		float angle = determinePose(laserScanData->ranges[middleSearchIndex-RANGE], (-1.5708 + (middleSearchIndex-RANGE)*laserScanData->angle_increment), laserScanData->ranges[middleSearchIndex], (-1.5708 + middleSearchIndex*laserScanData->angle_increment), laserScanData->ranges[middleSearchIndex+RANGE], (-1.5708 + (middleSearchIndex+RANGE)*laserScanData->angle_increment));
		float x = 0;
		float y = 0;
		if (angle < ANGLE_90) {
			// Check if the lengths are uneven using cosine rule - set a tolerance
			x = cosineRule(laserScanData->ranges[startSearchIndex+1], laserScanData->ranges[middleSearchIndex], (middleSearchIndex-startSearchIndex-1)*laserScanData->angle_increment);
			y = cosineRule(laserScanData->ranges[endSearchIndex], laserScanData->ranges[middleSearchIndex], (endSearchIndex-middleSearchIndex)*laserScanData->angle_increment);
			cout << "x: " << laserScanData->ranges[endSearchIndex] << " y: " << laserScanData->ranges[middleSearchIndex] << "\n";
			cout << "Tolerance Checking: \n";
			if ((x > y-TOLERANCE) && (x < y+TOLERANCE)) {
				// Equal lengths - this is square
				cout << "Found a square\n";
			} else {
				cout << "Found a rectangle\n";
			}
		} else if (angle > ANGLE_180) {
			// Determine the gradient changes
			if (constantGradient) {
				// Potential for vertex of square OR rectangle therefore unsure
				cout << "More Info Required!\n";
			} else {
				cout << "Found a circle\n";
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
	
	//cout << "distancea: " << xA << "\n";
	//cout << "distanceb: " << xB << "\n";
	//cout << "distancec: " << xC << "\n";
	//cout << "distancea: " << yA << "\n";
	//cout << "distanceb: " << yB << "\n";
	//cout << "distancec: " << yC << "\n";
	
	Coord A;
	Coord C;
	
	A.x = xA - xB;
	A.y = -(abs(yA-yB));
	C.x = xC - xB;
	C.y = abs(yC-yB);

	//cout << "Ax: " << A.x << "\n";
	//cout << "Ay: " << A.y << "\n";
	//cout << "Cx: " << C.x << "\n";
	//cout << "Cy: " << C.y << "\n";
	cout << "Angle: " << cross(A, C) << "\n";
	cout << (yA-yB)/abs(xA-xB) << "\n";
	cout << (yB-yC)/abs(xB-xC) << "\n";
	if ((yA-yB)/abs(xA-xB) == (yB-yC)/abs(xB-xC)) {
		// Constant gradient detected
		constantGradient = true;
	}

	return abs(cross(A,C));
}

float cross(Coord A, Coord C) {
	float dot = A.x*C.x + A.y*C.y;
	float det = A.y*C.x - A.x*C.y;
	return atan2(det, dot);
}

float cosineRule(float distanceA, float distanceB, float angleC) { 
	return sqrt(pow(distanceA, 2.0) + pow(distanceB, 2.0) - 2*distanceA*distanceB*cos(angleC));
}


