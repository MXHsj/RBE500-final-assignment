/*
 * SCARA_client.cpp
 *
 *  Created on: Oct 3, 2019
 *      Author: xihan
 */

#include "ros/ros.h"
#include "rbe500fp_part1/calcIK.h"
#include <cstdlib>

#define deg2rad 3.1415/180

using namespace std;

int main(int argc, char **argv)
{

  //UPDATE FOR INVERSE KINEMATICS
  ros::init(argc, argv, "SCARAIK_client");

  if (argc != 4)
  {
    ROS_INFO("usage: calculate inverse kinematics given [x,y,z] in [m], [roll,pitch,yaw] in [degrees]");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client1 = n.serviceClient<rbe500fp_part1::calcIK>("calculate_IK");

  rbe500fp_part1::calcIK srv1;

  srv1.request.x = atoll(argv[1]);
  srv1.request.y = atoll(argv[2]);
  srv1.request.z = atoll(argv[3]);
  srv1.request.roll = atoll(argv[4]);
  srv1.request.pitch = atoll(argv[5]);
  srv1.request.yaw = atoll(argv[6]);

  if (client1.call(srv1))
  {
    ROS_INFO("inverse kinematics: \n Tip pose = ");

    cout<<(float)srv1.response.q1<<"\n";
    cout<<(float)srv1.response.q2<<"\n";
    cout<<(float)srv1.response.q3<<"\n";
    
  }
  else
  {
    ROS_ERROR("Failed to call service calcIK");
    return 1;
  }

  return 0;
}


