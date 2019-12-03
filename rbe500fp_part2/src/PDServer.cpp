/*
 * PDServer.cpp
 *
 *  Created on: Nov 25, 2019
 */

#include "ros/ros.h"
#include "rbe500fp_part2/RefPos.h"
#include "std_msgs/Float64.h"
#include <boost/bind.hpp>
#include <cmath>

# define L1 0.2		//m
# define L2 0.2		//m
# define L3 0.2		//m
# define L4 0.1 	//m

using namespace std;

ros::Publisher command_pub;

// Recieve a Reference Position 
bool goTo(rbe500fp_part2::RefPos::Request &req, rbe500fp_part2::RefPos::Response &res)
{
	
  std_msgs::Float64 message;
  message.data = req.ref;
  
  // Publish the reference position to the Gazebo topic
  command_pub.publish(message);

	ROS_INFO("Request: Reference = %f", (float)req.ref);
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

  // Create a publish relation to the SCARA command topic
  command_pub = n.advertise<std_msgs::Float64>("/custom_scara/joint3_position_controller/command", 1000);

  ROS_INFO("Ready to Move the Third Joint");

  ros::spin();

  return 0;
}


