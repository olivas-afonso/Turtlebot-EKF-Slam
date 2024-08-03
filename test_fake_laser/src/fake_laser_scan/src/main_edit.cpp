//Author: Nicolas Gallardo
//Date:7/30/17
//Filename: main.cpp
//This us the main file that utilizes the created classes

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

//global variables:
double g_PI = 3.14159265359;
#define g_SAMPLES_PER_REV 100
double g_LASER_FREQ = 40;
double ranges[g_SAMPLES_PER_REV];
double intensities[g_SAMPLES_PER_REV];

std_msgs::Float32 current_scan_time;
std_msgs::Float32 last_scan_time;

int main(int argc, char **argv)
{
//---------------start-up---------------------------

	ros::init(argc, argv, "laser_scan_publisher");
	ros::NodeHandle n;
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);
	ros::Rate r(10);
	current_scan_time.data = 0.0;
	last_scan_time.data = 0.0;
	int count = 0;

//--------------------------------------------------
while(ros::ok())
{
    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < g_SAMPLES_PER_REV; ++i){
      ranges[i] = count;
      intensities[i] = 100 + count;
    }


	ros::Time scan_time = ros::Time::now();
	sensor_msgs::LaserScan scan;
 	scan.header.stamp = scan_time;
    	scan.header.frame_id = "laser_frame";
	current_scan_time.data = static_cast<float>(ros::Time::now().toSec());
	scan.angle_min = -1.57;
	scan.angle_max = 1.57;
	scan.angle_increment = g_PI / g_SAMPLES_PER_REV;
	scan.time_increment = (1 / g_LASER_FREQ) / (g_SAMPLES_PER_REV);;
	scan.scan_time = current_scan_time.data - last_scan_time.data;
	scan.range_min = 0;
	scan.range_max = 100;

    	scan.ranges.resize(g_SAMPLES_PER_REV);
    	scan.intensities.resize(g_SAMPLES_PER_REV);
	for(int i = 0; i < g_SAMPLES_PER_REV ; i++)
	{
		scan.ranges[i] = ranges[i];
		scan.intensities[i] = intensities[i];
		
	}
	
	last_scan_time.data = static_cast<float>(ros::Time::now().toSec());

	scan_pub.publish(scan);
	++count;
	r.sleep();

}
return 0;
}
