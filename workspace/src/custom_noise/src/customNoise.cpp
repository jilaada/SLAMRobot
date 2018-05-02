#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_listener.h"
#include <ctime>
#include <math.h>
#include <cmath> 
#include "random_numbers/random_numbers.h"

double mean = 0;
double stddev = 0.03;

ros::Publisher laser_pub_object;
random_numbers::RandomNumberGenerator gen;
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{
	sensor_msgs::LaserScan output = *laserScanData;
	// Example of using some of the non-range data-types
	float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min)  / (laserScanData->angle_increment);
//	 
	for(int j = 0; j < rangeDataNum; ++j)	// Go through the laser data
	{
		///linear random
		double dropout = (rand()/(RAND_MAX + 1.0));
		if (dropout > 0.1) {
			//gaussian random
			double random = gen.gaussian(mean, stddev);
			output.ranges[j] = laserScanData->ranges[j]+random;
		}else{
			double random = gen.gaussian(mean, stddev*10);
			output.ranges[j] = laserScanData->ranges[j]+random;
		}
	}
	
	laser_pub_object.publish(output);
}

int main (int argc, char **argv)
{	
	ros::init(argc, argv, "noise_node");	// command line ROS arguments
	ros::NodeHandle my_handle;	// ROS comms access point
	laser_pub_object = my_handle.advertise<sensor_msgs::LaserScan>("/scan_noise",1);
	ros::Subscriber laser_sub_object = my_handle.subscribe("/scanOrig", 1, laserScanCallback);
	srand(time(NULL));
	ros::spin();

	return 0;
}

