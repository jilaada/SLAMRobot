#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// Libraries for IO read write operations
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

// Including helper functions
#include "globals.h"

enum detectionType { 
	noEdge,
	fallingEdge,
	risingEdge
};

enum ShapeType {
	CIRCLE,
	RECTANGLE,
	SQUARE,
	MORE_INFO
};

struct Pose {
	float poseX;
	float poseY;
	double roll;
	double pitch;
	double yaw;
};

struct Coord {
	float x;
	float y;
};
	
