/*
 * SCARA_client.cpp
 *
 *  Created on: Oct 3, 2019
 *      Author: xihan
 */

#include "ros/ros.h"
#include "rbe500fp_part1/calcIK.h"
#include "rbe500fp_part1/calcFK.h"
#include "control_msgs/JointControllerState.h"
#include <cstdlib>

#define rad2deg 180/3.1415

using namespace std;

class Gazebo_Listener{
  public:
    float q1;
    float q2;
    float q3;
    void update_jointOne(const control_msgs::JointControllerState::ConstPtr &msg);
    void update_jointTwo(const control_msgs::JointControllerState::ConstPtr &msg);
    void update_jointThree(const control_msgs::JointControllerState::ConstPtr &msg);
    float get_q2();
};

void Gazebo_Listener::update_jointOne(const control_msgs::JointControllerState::ConstPtr &msg)
{
  q1 = msg->set_point;
}
void Gazebo_Listener::update_jointTwo(const control_msgs::JointControllerState::ConstPtr &msg)
{
  q2 = msg->set_point;
}
void Gazebo_Listener::update_jointThree(const control_msgs::JointControllerState::ConstPtr &msg)
{
  q3 = msg->set_point;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SCARA_Gazebo_client");
  ros::NodeHandle n;
  ros::Rate loop_rate(30);
  Gazebo_Listener g_listener;

  ros::Subscriber jointOne = n.subscribe("/custom_scara/joint1_position_controller/state", 1, &Gazebo_Listener::update_jointOne, &g_listener);
  ros::Subscriber jointTwo = n.subscribe("/custom_scara/joint2_position_controller/state", 1, &Gazebo_Listener::update_jointTwo, &g_listener);
  ros::Subscriber jointThree = n.subscribe("/custom_scara/joint3_position_controller/state", 1, &Gazebo_Listener::update_jointThree, &g_listener);
  
  ros::ServiceClient IKclient = n.serviceClient<rbe500fp_part1::calcIK>("calculate_IK");
  ros::ServiceClient FKclient = n.serviceClient<rbe500fp_part1::calcFK>("calculate_FK");

  rbe500fp_part1::calcIK IKsrv;
  rbe500fp_part1::calcFK FKsrv;

  int count = 0;
  while(count < 3)
  {
    ros::spinOnce();
    cout << "Q1: " << g_listener.q1 << endl;
    cout << "Q2: " << g_listener.q2 << endl;
    cout << "Q3: " << g_listener.q3 << endl;
    loop_rate.sleep();
    count++;
  }  


  FKsrv.request.q1 = g_listener.q1;
  FKsrv.request.q2 = g_listener.q2;
  FKsrv.request.q3 = g_listener.q3;

  cout << "FKsrc.request.q1: " << FKsrv.request.q1 << endl;
  cout << "FKsrc.request.q2: " << FKsrv.request.q2 << endl;
  cout << "FKsrc.request.q2: " << FKsrv.request.q3 << endl;


  
  /*
  if (FKclient.call(FKsrv))
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
  */
  // if (client1.call(srv1))
  // {
  //   ROS_INFO("inverse kinematics: \n");
  //   cout << "elbow_down solution:" << endl;
  //   cout << "q1 = " << (float)srv1.response.q1[0]*rad2deg << " [deg]" << endl;
  //   cout << "q2 = " << (float)srv1.response.q2[0]*rad2deg << " [deg]" << endl;
  //   cout << "q3 = " << (float)srv1.response.q3[0] << " [m]" << endl;

  //   cout << "\nelbow-up solution:" << endl;
  //   cout << "q1 = " << (float)srv1.response.q1[1]*rad2deg << " [deg]" << endl;
  //   cout << "q2 = " << (float)srv1.response.q2[1]*rad2deg << " [deg]" << endl;
  //   cout << "q3 = " << (float)srv1.response.q3[1] << " [m]" << endl;
  // }
  // else
  // {
  //   ROS_ERROR("Failed to call service calcIK");
  //   return 1;
  // }
  
  ros::spin();
  return 0;
}


