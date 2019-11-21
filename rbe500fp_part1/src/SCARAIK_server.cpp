/*
 * SCARAIK_server.cpp
 *
 *  Created on: Nov 21, 2019
 */

#include "ros/ros.h"
#include "rbe500fp_part1/calcIK.h"
#include <cmath>

#define rad2deg 180/3.1415
#define pi 3.1415

# define L1 0.2		//m
# define L2 0.2		//m
# define L3 0.2		//m
# define L4 0.1 	//m

using namespace std;

// calculate inverse kinematics
bool fwdkin(rbe500fp_part1::calcIK::Request &req, rbe500fp_part1::calcIK::Response &res)
{
	float tolerence = 0.0001;
	res.q1.resize(2);
	res.q2.resize(2);
	res.q3.resize(2);

	// elbow-down
	res.q2[0] = acos((req.x*req.x+req.y*req.y-L2*L2-L3*L3)/(2*L2*L3));
	res.q1[0] = atan2(req.y,req.x)-atan2(L3*sin(res.q2[0]),(L2+L3*cos(res.q2[0])));
	res.q3[0] = (L4 + req.z) - L1;

	if(std::abs(res.q1[0]) < tolerence)
		res.q1[0] = 0;
	if(std::abs(res.q2[0]) < tolerence)
		res.q2[0] = 0;
	if(std::abs(res.q3[0]) < tolerence)
		res.q3[0] = 0;

	// elbow-up
	res.q2[1] = acos((L2*L2+L3*L3-req.x*req.x-req.y*req.y)/(2*L2*L3)) - pi;
	res.q1[1] = atan2(req.y,req.x)+atan2(2*L2*L3*sin(pi+res.q2[1]),(L2*L2-L3*L3+req.x*req.x+req.y*req.y));
	res.q3[1] = (L4 +req.z) - L1;

	if(std::abs(res.q1[1]) < tolerence)
		res.q1[1] = 0;
	if(std::abs(res.q2[1]) < tolerence)
		res.q2[1] = 0;
	if(std::abs(res.q3[1]) < tolerence)
		res.q3[1] = 0;
	
	ROS_INFO("request: x=%f, y=%f, z=%f", (float)req.x, (float)req.y, (float)req.z);

	ROS_INFO("sending back response \n");
	cout << "elbow-down solution:" << endl;
	cout << "q1 = " << (float)res.q1[0]*rad2deg << " [deg]" << endl;
	cout << "q2 = " << (float)res.q2[0]*rad2deg << " [deg]" << endl;
	cout << "q3 = " << (float)res.q3[0] << " [m]" << endl;

	cout << "\nelbow-up solution:" << endl;
	cout << "q1 = " << (float)res.q1[1]*rad2deg << " [deg]" << endl;
	cout << "q2 = " << (float)res.q2[1]*rad2deg << " [deg]" << endl;
	cout << "q3 = " << (float)res.q3[1] << " [m]" << endl;

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


