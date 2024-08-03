//Author: Nicolas Gallardo
//Date:7/30/17
//Filename: fake_scan_node.cpp
//This cpp file implements the classes and functions required to produce and publish fake laser scans
//as a ROS LaserScan message

//ROS includes:

#include "fake_scan_node.h"

FakeLaserScan laserScan;
sensor_msgs::LaserScan scan;

void FakeLaserScan::begin()
{
	//create ros node and publishers
	ros::init(argc, argv, "laser_scan_publisher");
	ros::NodeHandle n;
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);
	srand(ros::Time::now());
	ros::Rate r(10);
	
}

void FakeLaserScan::fakeScanPublisher()
{
	current_scan_time = ros::Time::now();
	scan.angle_min = 0.0;
	scan.angle_max = 2 * g_PI;
	scan.angle_increment = 2 * g_PI / g_SAMPLES_PER_REV;
	scan.time_increment = (1 / g_LASER_FREQ) / (g_SAMPLES_PER_REV);;
	scan.scan_time = current_scan_time - last_scan_time;
	scan.range_min = 0.5;
	scan.range_max = 6.0;
	for(int i = 0; i < g_SAMPLES_PER_REV ; i++)
	{
		scan.ranges[i] = rand() % 7;
		
	}
	scan.intensities[g_SAMPLES_PER_REV] = [0.0];
	last_scan_time = ros::Time::now();
	r.sleep();
	
}