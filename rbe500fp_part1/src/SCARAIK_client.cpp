/*
 * SCARA_client.cpp
 *
 *  Created on: Oct 3, 2019
 *      Author: xihan
 */

#include "ros/ros.h"
#include "rbe500fp_part1/calcIK.h"
#include <cstdlib>

#define rad2deg 180/3.1415

using namespace std;

int main(int argc, char **argv)
{

  //UPDATE FOR INVERSE KINEMATICS
  ros::init(argc, argv, "SCARAIK_client");

  if (argc != 7)
  {
    ROS_INFO("usage: calculate inverse kinematics given [x,y,z] in [m], [roll,pitch,yaw] in [degrees]");
    std::cout << "number of input arguments:" << argc << std::endl;
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client1 = n.serviceClient<rbe500fp_part1::calcIK>("calculate_IK");

  rbe500fp_part1::calcIK srv1;

  srv1.request.x = atof(argv[1]);
  srv1.request.y = atof(argv[2]);
  srv1.request.z = atof(argv[3]);
  srv1.request.roll = atof(argv[4]);
  srv1.request.pitch = atof(argv[5]);
  srv1.request.yaw = atof(argv[6]);

  if (client1.call(srv1))
  {
    ROS_INFO("inverse kinematics: \n");
    cout << "elbow_down solution:" << endl;
    cout << "q1 = " << (float)srv1.response.q1[0]*rad2deg << " [deg]" << endl;
    cout << "q2 = " << (float)srv1.response.q2[0]*rad2deg << " [deg]" << endl;
    cout << "q3 = " << (float)srv1.response.q3[0] << " [m]" << endl;

    cout << "\nelbow-up solution:" << endl;
    cout << "q1 = " << (float)srv1.response.q1[1]*rad2deg << " [deg]" << endl;
    cout << "q2 = " << (float)srv1.response.q2[1]*rad2deg << " [deg]" << endl;
    cout << "q3 = " << (float)srv1.response.q3[1] << " [m]" << endl;
  }
  else
  {
    ROS_ERROR("Failed to call service calcIK");
    return 1;
  }

  return 0;
}


