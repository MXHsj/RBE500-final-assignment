/*
 * SCARA_FK.cpp
 *
 *  Created on: Oct 2, 2019
 *      Author: xihan
 */

#include "ros/ros.h"
#include "rbe500fp_part1/calcFK.h"
#include <cmath>

# define L1 0.2		//m
# define L2 0.2		//m
# define L3 0.2		//m
# define L4 0.1 	//m

// calculate forward kinematics
bool fwdkin(rbe500fp_part1::calcFK::Request &req, rbe500fp_part1::calcFK::Response &res)
{
	res.T.resize(6);
	float tolerence = 0.001;

	res.T[0] = L2*cos(req.q1) + L3*cos(req.q1 + req.q2);
	res.T[1] = L2*sin(req.q1) + L3*sin(req.q1 + req.q2);
	res.T[2] = L1 - L4 - req.q3;
	res.T[3] = 0;
	res.T[4] = 0;
	float Tpos0 = cos(req.q1 + req.q2);
	float Tpos3 = sin(req.q1 + req.q2);
	res.T[5] = atan2(Tpos3, Tpos0);

	ROS_INFO("request: q1=%f, q2=%f, q3=%f", (float)req.q1, (float)req.q2, (float)req.q3);
	ROS_INFO("sending back response T:");

	for(int i = 0; i < 6; i ++)
	{
		if(std::abs(res.T[i]) <= tolerence)
			res.T[i] = 0;

		std::cout << (float)res.T[i] << "\n";
	}

	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SCARAFK_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("calculate_FK", fwdkin);

  ROS_INFO("Ready to calculate forward kinematics for SCARA robot");

  ros::spin();

  return 0;
}


