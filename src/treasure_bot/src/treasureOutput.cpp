#include <ros/ros.h>
#include <string>
#include <vector>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <logical_camera_plugin/logicalImage.h>

float rx = 0;
float ry = 0;
float rz = 0;
std::vector<std::string> names;
geometry_msgs::Quaternion ori;

void print(float x, float y, float z, std::string name, geometry_msgs::Quaternion o){
	double angle = tf::getYaw(o);
	angle = angles::normalize_angle_positive(angle);
	double degree = angle*180/M_PI;
	if((degree >= 0) && (degree < 90)){
		x = x - rx;
		y = y - ry;
	}
	else if((degree >= 90) && (degree < 180)){
		x = x + rx;
		y = y - ry;
	}
	else if((degree >= 180) && (degree < 270)){
		x = x + rx;
		y = y + ry;
	}
	else {
		x = x - rx;
		y = y + ry;
	}
	ROS_INFO_STREAM("Name: " << name);
	ROS_INFO_STREAM("X location: " << x);
	ROS_INFO_STREAM("Y location: " << y);
	ROS_INFO_STREAM("Z location: " << z);
	ROS_INFO_STREAM("Orientation: " << degree);

}

void gotCamMsg(const logical_camera_plugin::logicalImage &msg){

	std::string treasureName = msg.modelName;

	float x = msg.pose_pos_x;
	float y = msg.pose_pos_y;
	float z = msg.pose_pos_z;

	geometry_msgs::Quaternion q;
	q.x = msg.pose_rot_x;
	q.y = msg.pose_rot_y;
	q.z = msg.pose_rot_z;
	q.w = msg.pose_rot_w;

	if(names.empty()){
		names.push_back(treasureName);
		print(x, y, z, treasureName, q);
	}

	else {
		bool found = false;
		for(int i=0; i < names.size(); i++){
			std::string knownTreasure = names.at(i);
			if (!names.at(i).compare(treasureName)) {
				found = true;
				break;
			}
		}
		if(!found) {
			names.push_back(treasureName);
			print(x, y, z, treasureName, q);
		}
	}
}

void gotLocation(const geometry_msgs::PoseWithCovarianceStamped &msg){
	rx = msg.pose.pose.position.x;
	ry = msg.pose.pose.position.y;
	rz = msg.pose.pose.position.z;
	ori = msg.pose.pose.orientation;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "treasuretracker");
	ros::NodeHandle nh;
	ros::Subscriber camSUb = nh.subscribe("/objectsDetected",1000, &gotCamMsg);
	ros::Subscriber locSub = nh.subscribe("/amcl_pose", 1000, &gotLocation);
	ros::spin();
}

