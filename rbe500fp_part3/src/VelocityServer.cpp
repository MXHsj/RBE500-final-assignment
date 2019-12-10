/*
 * VelocityServer.cpp
 *
 *  Created on: Nov 25, 2019
 */

#include "ros/ros.h"
#include "rbe500fp_part3/RefVel.h"
#include "std_msgs/Float64.h"
#include <boost/bind.hpp>
#include <cmath>

# define L1 0.2		//m
# define L2 0.2		//m
# define L3 0.2		//m
# define L4 0.1 	//m

using namespace std;

ros::Publisher command_pub_j1;
ros::Publisher command_pub_j2;
ros::Publisher command_pub_j3;

class Gazebo_Listener{
public:
    float q1;
    float q2;
    float q3;
    void update_jointOne(const control_msgs::JointControllerState::ConstPtr &msg);
    void update_jointTwo(const control_msgs::JointControllerState::ConstPtr &msg);
    void update_jointThree(const control_msgs::JointControllerState::ConstPtr &msg);
};

void Gazebo_Listener::update_jointOne(const control_msgs::JointControllerState::ConstPtr &msg)
{
    q1 = msg->set_point;
    if (std::abs(q1) < tolerence)
        q1 = 0;
}
void Gazebo_Listener::update_jointTwo(const control_msgs::JointControllerState::ConstPtr &msg)
{
    q2 = msg->set_point;
    if (std::abs(q2) < tolerence)
        q2 = 0;
}
void Gazebo_Listener::update_jointThree(const control_msgs::JointControllerState::ConstPtr &msg)
{
    q3 = msg->set_point;
    if (std::abs(q3) < tolerence)
        q3 = 0;
}



// Recieve a Reference Position 
bool goTo(rbe500fp_part3::RefVel::Request &req, rbe500fp_part3::RefVel::Response &res)
{
	

  //Create Client
  ros::ServiceClient KinClient = n.serviceClient<rbe500fp_part3::InvVelKin>("calculate_IK");

  rbe500fp_part3::InvVelKin KinSrv;

  /*
  float32 dx    linear velocity in x direction
  float32 dy    linear velocity in y direction
  float32 dz    linear velocity in z direction
  float32 dax   angular velocity about x
  float32 day   angular velocity about y
  float32 daz   angular velcotiy about z
  float32 q1    Joint 1 angle [rad]
  float32 q2    Joint 2 angle [rad]
  float32 q3    Joint 3 position [m]
  */

  RefSrv.request.dx = req.refx;
  RefSrv.request.dy = req.refy;
  RefSrv.request.dz = req.refz;
  RefSrv.request.dax = 0;
  RefSrv.request.day = 0;
  RefSrv.request.daz = 0;

  Gazebo_Listener g_listener;


  ros::Subscriber jointOne = n.subscribe("/custom_scara/joint1_position_controller/state", 1, &Gazebo_Listener::update_jointOne, &g_listener);
  ros::Subscriber jointTwo = n.subscribe("/custom_scara/joint2_position_controller/state", 1, &Gazebo_Listener::update_jointTwo, &g_listener);
  ros::Subscriber jointThree = n.subscribe("/custom_scara/joint3_position_controller/state", 1, &Gazebo_Listener::update_jointThree, &g_listener);
  
  for (int i = 0; i < 3; i ++)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  RefSrv.request.q1 = g_listener.q1;
  RefSrv.request.q2 = g_listener.q2;
  RefSrv.request.q3 = g_listener.q3;


  // Define messages to publish to controller
  std_msgs::Float64 messageJoint1;
  std_msgs::Float64 messageJoint2;
  std_msgs::Float64 messageJoint3;


  if (RefClient.call(KinSrv))
  {
    {
      ROS_INFO("Moving: \n Motion = ");
      if (KinSrv.response.done)
      {
          ROS_INFO("Complete");
          // Set values of messages for each joint from response of kinematics server
          messageJoint1.data = req.dq1;
          messageJoint2.data = req.dq2;
          messageJoint3.data = req.dq3;
      }
      else
      {
          ROS_INFO("Incomplete");
      }
    }
    else
    {
        ROS_ERROR("Failed to call service RefSrv");
        return 1;
    }
  }
 
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
  ros::init(argc, argv, "Velocity_Server");
  ros::NodeHandle n;
  ros::Rate loop_rate(30);
    
  // Create a service for recieving reference position
  ros::ServiceServer service = n.advertiseService("VelServer", goTo);

  // Create a publish relation to the SCARA command topic
  /***********************************
    THIS NEEDS TO BE FILLED IN LATER
  ************************************/
  command_pub_j1 = n.advertise<std_msgs::Float64>("/FILL IN HERE", 1000);
  command_pub_j2 = n.advertise<std_msgs::Float64>("/FILL IN HERE", 1000);
  command_pub_j3 = n.advertise<std_msgs::Float64>("/FILL IN HERE", 1000);

  ROS_INFO("Ready to Move the SCARA Robot");

  ros::spin();

  return 0;
}


