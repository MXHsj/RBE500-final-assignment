/*
 * PDServer.cpp
 *
 *  Created on: Dec 10, 2019
 */

#include "ros/ros.h"
#include "rbe500fp_part3/RefPos.h"
#include "std_msgs/Float64.h"
#include <boost/bind.hpp>
#include <cmath>

# define L1 0.2		//m
# define L2 0.2		//m
# define L3 0.2		//m
# define L4 0.1 	//m

using namespace std;

// Global variables for publishing joint commands
ros::Publisher command_pub_j1;
ros::Publisher command_pub_j2;
ros::Publisher command_pub_j3;

// Recieve a Reference Position 
bool goTo(rbe500fp_part3::RefPos::Request &req, rbe500fp_part3::RefPos::Response &res)
{
  
  // Make Float64 messages for each joint
  std_msgs::Float64 messageJoint1;
  messageJoint1.data = req.ref1;
  std_msgs::Float64 messageJoint2;
  messageJoint2.data = req.ref2;
  std_msgs::Float64 messageJoint3;
  messageJoint3.data = req.ref3;
  
  // Publish the reference position to the Gazebo topic
  command_pub_j1.publish(messageJoint1);
  command_pub_j2.publish(messageJoint2);
  command_pub_j3.publish(messageJoint3);

	ROS_INFO("Request: Joint 1 Reference = %f, Joint 2 Reference = %f, Joint 3 Reference = %f", (float)req.ref1, (float)req.ref2, (float)req.ref3);
	ROS_INFO("Sending Back Response: Complete");

  res.done = 1;

	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PD_Server");
  ros::NodeHandle n;

  // Create a service for recieving reference position
  ros::ServiceServer service = n.advertiseService("PD_Server", goTo);

  // Create a publish relation to the SCARA command topic for each joint
  command_pub_j1 = n.advertise<std_msgs::Float64>("/custom_scara/joint1_position_controller/command", 1000);
  command_pub_j2 = n.advertise<std_msgs::Float64>("/custom_scara/joint2_position_controller/command", 1000);
  command_pub_j3 = n.advertise<std_msgs::Float64>("/custom_scara/joint3_position_controller/command", 1000);


  ROS_INFO("Ready to Move the SCARA Robot");

  ros::spin();

  return 0;
}


