#include "goal.h"
#include <ros/ros.h>
#include <queue>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseAction.h>

using namespace std;
queue<goal> goals;

void gotPose(const geometry_msgs::Pose2D &msg){
	goal newGoal;
	newGoal.x = msg.x;
	newGoal.y = msg.y;
	newGoal.theta = msg.theta;
	goals.push(newGoal);
}

int main(int argv, char ** argc) {
	ros::init(argv, argc, "pathing");
	ros::NodeHandle nh;
	ros::Rate rate(10);
	ros::Subscriber poseSub = nh.subscribe("targetpose", 1000, &gotPose);
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
	move_base_msgs::MoveBaseGoal nextGoal;
	// Wait for it ...
	while(!ac.waitForServer()){}
	while(ros::ok()) {
		if(!goals.empty()) {
			// Pull the highest priority goal
			goal target = goals.front();
			goals.pop();
			// Construct a new goal
			tf::Quaternion q;
			q = tf::createQuaternionFromYaw(target.theta);
			geometry_msgs::Quaternion qm;
			tf::quaternionTFToMsg(q, qm);
			nextGoal.target_pose.pose.orientation = qm; 
			nextGoal.target_pose.header.frame_id = "map";
			nextGoal.target_pose.header.stamp = ros::Time::now();
			nextGoal.target_pose.pose.position.x = target.x;
			nextGoal.target_pose.pose.position.y = target.y;
			// Send it!
			ac.sendGoal(nextGoal);
			
			ros::Duration timeout (25.0);
			if (!ac.waitForResult(timeout)) {
				ROS_INFO_STREAM ("timeout.");
				ac.cancelGoal();
			}
			
			if(!(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)){
				ROS_INFO_STREAM("I have failed to get to the goal, sir.");
				ac.cancelGoal();
			}
			else ROS_INFO_STREAM("Successfully reached goal, sir!");
		}
		rate.sleep();
		ros::spinOnce();
	}

}
