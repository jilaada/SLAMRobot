#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

// Libraries for IO read write operations
#include <iostream>
#include <fstream>
#include <cmath>

// Including helper functions
#include "globals.h"

enum detectionType { 
	noEdge,
	fallingEdge,
	risingEdge
};
	
