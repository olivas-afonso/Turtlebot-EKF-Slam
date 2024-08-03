//Author: Nicolas Gallardo
//Date:7/30/17
//This header file defines the classes and functions required to produce and publish fake laser scans
//a as ROS LaserScan message

#ifndef FAKE_SCAN_NODE_H_
#define FAKE_SCAN_NODE_H_

//ROS includes:

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

//global variables:
#define g_PI 3.14159265359
#define g_SAMPLES_PER_REV 400
#define g_LASER_FREQ 10

class FakeLaserScan{
	
	public:
	FakeLaserScan(){};
	void begin();
	void fakeScanPublisher();
	
	private:
	float32 angle_min;
	float32 angle_max;
	float32 angle_increment;
	float32 time_increment;
	float32 scan_time;
	float32 range_min;
	float32 range_max;
	float32[] ranges;
	float32[] intensities;
	float32 last_scan_time;
	float32 current_scan_time;
	
}

extern FakeLaserScan laserScan;

#endif /* FAKE_SCAN_NODE_H_ */