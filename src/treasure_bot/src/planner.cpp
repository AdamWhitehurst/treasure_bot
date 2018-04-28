#include <ros/ros.h>
#include <stdlib.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pubVel;
ros::Time start;
geometry_msgs::Twist msg;

void gotMap(const nav_msgs::OccupancyGrid &map) {
	ROS_INFO("-----------------------\n");
	ROS_INFO_STREAM(map);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;
  pubVel = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
  ros::Subscriber subMap = nh.subscribe("/map", 1000, &gotMap);
  ros::Rate rate(10);
}
