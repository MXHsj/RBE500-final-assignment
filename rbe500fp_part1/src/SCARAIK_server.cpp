/*
 * SCARA_FK.cpp
 *
 *  Created on: Oct 2, 2019
 *      Author: xihan
 */

#include "ros/ros.h"
#include "rbe500fp_part1/calcIK.h"
#include <cmath>

# define L1 0.2		//m
# define L2 0.2		//m
# define L3 0.2		//m
# define L4 0.1 	//m

// calculate inverse kinematics
bool fwdkin(rbe500fp_part1::calcIK::Request &req, rbe500fp_part1::calcIK::Response &res)
{
	//UPDATE FOR INVERSE KINEMATICS
	float tolerence = 0.001;

	res.q1 = atan2(req.y,req.x);
	res.q2 = acos((req.x*req.x + req.y*req.y - L2*L2 - L3*L3)/(2*L2*L3));
	res.q3 = L1 - req.z;

	if(std::abs(res.q1) < tolerence)
		res.q1 = 0;

	if(std::abs(res.q2) < tolerence)
		res.q1 = 0;

	if(std::abs(res.q3) < tolerence)
		res.q1 = 0;
	
	ROS_INFO("request: x=%f, y=%f, z=%f", (float)req.x, (float)req.y, (float)req.z);
	ROS_INFO("sending back response T:");

	std::cout << (float)res.q1 << "\n";
	std::cout << (float)res.q2 << "\n";
	std::cout << (float)res.q3 << "\n";

	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SCARAIK_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("calculate_IK", fwdkin);

  ROS_INFO("Ready to calculate inverse kinematics for SCARA robot");

  ros::spin();

  return 0;
}


