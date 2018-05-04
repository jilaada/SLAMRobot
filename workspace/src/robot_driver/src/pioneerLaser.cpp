// Includes for multiple files
#include "includes.h"

using namespace std;

geometry_msgs::Twist velocityCommand; 
void incrementPointer(int &pointer);

/*
The scan subscriber call back function
To understand the sensor_msgs::LaserScan object look at
http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
*/
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData) {
	// Compute the number of data points
	// max angle and min angle of the laser scanner divide by the increment angle of each data point
	float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);
	// Rotating buffer array
	int bufferStatus[BUFFER_SIZE] = { };
	int bufferPointer = 0;
	detectionType currentType = noEdge;
	detectionType previousType = noEdge;
	bool detecting = false;
	int startSearchIndex = 0;
	int endSearchIndex = 0;
	
	// 
	int detectedIndex = 0;
	
	float currentAngle = 3.14;
	
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
				
			}
			
			if ((currentStatus == 3)) {
				// Object detected
				// Eventually this will need to be placed into a linked list for defined searching
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
			cout << currentStatus << " : " << laserScanData->ranges[j] << "\n"; 
		}
		bufferStatus[bufferPointer] = currentStatus;
		incrementPointer(bufferPointer);
		currentAngle = currentAngle - laserScanData->angle_increment;
	}
}

/*
The odometer call back function
http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
*/
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	//Print the odom data structure
	//Message sequence
	//ROS_INFO("Seq: [%d]", msg->header.seq);
	//Position
	//ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
	//Orientation
	//ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	//Velocity
	//ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
		
}

int main (int argc, char **argv) {	
	ros::init(argc, argv, "pioneer_laser_node");	// command line ROS arguments
	ros::NodeHandle my_handle;	// ROS comms access point

	ros::Publisher vel_pub_object = my_handle.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);

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
