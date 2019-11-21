/*
 * SCARAFK_client.cpp
 *
 *  Created on: Nov 21, 2019
 */

#include "ros/ros.h"
#include "rbe500fp_part1/calcFK.h"
#include <cstdlib>

#define deg2rad 3.1415/180

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SCARAFK_client");

  if (argc != 4)
  {
    ROS_INFO("usage: calculate forward kinematics and jacobians given [q1] [q2] in [degrees], [q3] in [m]");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client1 = n.serviceClient<rbe500fp_part1::calcFK>("calculate_FK");

  rbe500fp_part1::calcFK srv1;

  srv1.request.q1 = atof(argv[1])*deg2rad;
  srv1.request.q2 = atof(argv[2])*deg2rad;
  srv1.request.q3 = atof(argv[3]);

  if (client1.call(srv1))
  {
    ROS_INFO("foward kinematics: \n T = ");

    for(int i = 0; i < 6; i ++)
    {
    	cout<<(float)srv1.response.T[i]<<"\n";
    }
  }
  else
  {
    ROS_ERROR("Failed to call service calcFK");
    return 1;
  }

  return 0;
}


