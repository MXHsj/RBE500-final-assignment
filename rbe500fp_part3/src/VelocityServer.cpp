/*
 * VelocityServer.cpp
 *
 *  Created on: Nov 25, 2019
 */

#include "ros/ros.h"
#include "rbe500fp_part3/RefVel.h"
#include "rbe500fp_part3/InvVelKin.h"
#include "std_msgs/Float64.h"
#include <boost/bind.hpp>
#include "control_msgs/JointControllerState.h"
#include <cmath>

# define L1 0.2		//m
# define L2 0.2		//m
# define L3 0.2		//m
# define L4 0.1 	//m
# define loop 10.0

using namespace std;

ros::Publisher command_pub_j1;
ros::Publisher command_pub_j2;
ros::Publisher command_pub_j3;

ros::ServiceClient KinClient;

ros::Subscriber jointOne;
ros::Subscriber jointTwo;
ros::Subscriber jointThree;
  
class Gazebo_Listener{
public:
    float q1;
    float q2;
    float q3;
    bool made;
    void update_jointOne(const control_msgs::JointControllerState::ConstPtr &msg);
    void update_jointTwo(const control_msgs::JointControllerState::ConstPtr &msg);
    void update_jointThree(const control_msgs::JointControllerState::ConstPtr &msg);

    Gazebo_Listener(){}
    Gazebo_Listener(bool pass_made){
      made = pass_made;
    }
};

void Gazebo_Listener::update_jointOne(const control_msgs::JointControllerState::ConstPtr &msg)
{
  
    q1 = msg->set_point;
    if (std::abs(q1) < 0.01)
        q1 = 0;
}
void Gazebo_Listener::update_jointTwo(const control_msgs::JointControllerState::ConstPtr &msg)
{
    q2 = msg->set_point;
    if (std::abs(q2) < 0.01)
        q2 = 0;
}
void Gazebo_Listener::update_jointThree(const control_msgs::JointControllerState::ConstPtr &msg)
{
    q3 = msg->set_point;
    if (std::abs(q3) < 0.01)
        q3 = 0;
}

Gazebo_Listener g_listener;

// Recieve a Reference Position 
bool goTo(rbe500fp_part3::RefVel::Request &req, rbe500fp_part3::RefVel::Response &res)
{
	
  //Create Client

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

  KinSrv.request.dx = req.refx;
  KinSrv.request.dy = req.refy;
  KinSrv.request.dz = req.refz;
  KinSrv.request.dax = 0.1;
  KinSrv.request.day = 0.1;
  KinSrv.request.daz = 0.1;
  
    ros::Rate loop_rate(loop);

    for (int i = 0; i < 3; i ++)
    {
      ros::spinOnce();
      ros::Duration(0.2).sleep();
    }

  while(ros::ok())  
  {
    KinSrv.request.q1 = g_listener.q1;
    KinSrv.request.q2 = g_listener.q2;
    KinSrv.request.q3 = g_listener.q3;

    // Define messages to publish to controller
    std_msgs::Float64 messageJoint1;
    std_msgs::Float64 messageJoint2;
    std_msgs::Float64 messageJoint3;

    if (KinClient.call(KinSrv))
    {
      ROS_INFO("Calculating");
    
      // Set values of messages for each joint from response of kinematics server
      messageJoint1.data = KinSrv.response.dq1;
      messageJoint2.data = KinSrv.response.dq2;
      messageJoint3.data = KinSrv.response.dq3;

    }
   else
    {
      ROS_ERROR("Failed to call service RefSrv");
      return 1;
    }
    cout << "one over" << (float)(1/loop) <<endl;
    std_msgs::Float64 distance1;
    std_msgs::Float64 distance2;
    std_msgs::Float64 distance3;


    distance1.data = g_listener.q1 + (float)(messageJoint1.data * (1/loop));
    distance2.data = g_listener.q2 + (float)(messageJoint2.data * (1/loop));
    distance3.data = g_listener.q3 + (float)(messageJoint3.data * (1/loop));
    
    command_pub_j1.publish(distance1);
    command_pub_j2.publish(distance2);
    command_pub_j3.publish(distance3);

    ros::spinOnce();
    loop_rate.sleep();
  }
    

	ROS_INFO("Request: X Velocity = %f, Y Velocity = %f, Z Velocity = %f", (float)req.refx, (float)req.refy, (float)req.refz);
	ROS_INFO("Sending Back Response: Complete");

  res.done = 1;

	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Velocity_Server");
  ros::NodeHandle n;    
  // Create a service for recieving reference position
  ros::ServiceServer service = n.advertiseService("VelServer", goTo);
  
  // Create a publish relation to the SCARA command topic
  /***********************************
    THIS NEEDS TO BE FILLED IN LATER
  ************************************/
  command_pub_j1 = n.advertise<std_msgs::Float64>("/custom_scara/joint1_position_controller/command", 1000);
  command_pub_j2 = n.advertise<std_msgs::Float64>("/custom_scara/joint2_position_controller/command", 1000);
  command_pub_j3 = n.advertise<std_msgs::Float64>("/custom_scara/joint3_position_controller/command", 1000);

  KinClient = n.serviceClient<rbe500fp_part3::InvVelKin>("calculate_IK");
  
  jointOne = n.subscribe("/custom_scara/joint1_position_controller/state", 1, &Gazebo_Listener::update_jointOne, &g_listener);
  jointTwo = n.subscribe("/custom_scara/joint2_position_controller/state", 1, &Gazebo_Listener::update_jointTwo, &g_listener); 
  jointThree = n.subscribe("/custom_scara/joint3_position_controller/state", 1, &Gazebo_Listener::update_jointThree, &g_listener);


  ROS_INFO("Ready to Move the SCARA Robot");

  ros::spin();

  return 0;
}


