/*
 * toFile.cpp
 *
 *  Created on: Nov 25, 2019
 *     
 */

#include "ros/ros.h"
#include "rbe500fp_part2/calcIK.h"
#include "rbe500fp_part2/calcFK.h"
#include "control_msgs/JointControllerState.h"
#include <iostream>
#include <fstream>
#include <cstdlib>

#define rad2deg 180/3.1415
#define tolerence 0.0001

using namespace std;

class Gazebo_Listener{
public:
    float q1;
    float q2;
    float q3;
    float error1;
    float error2;
    float error3;
    float currentPos1;
    float currentPos2;
    float currentPos3;

    ofstream outFile;
    void update_jointOne(const control_msgs::JointControllerState::ConstPtr &msg);
    void update_jointTwo(const control_msgs::JointControllerState::ConstPtr &msg);
    void update_jointThree(const control_msgs::JointControllerState::ConstPtr &msg);


    Gazebo_Listener()
    {
        outFile.open("/home/ellie/catkin_ws/src/RBE500-final-assignment/rbe500fp_part2/src/output.txt");
    }
};

void Gazebo_Listener::update_jointOne(const control_msgs::JointControllerState::ConstPtr &msg)
{
    q1 = msg->set_point;
    if (std::abs(q1) < tolerence)
        q1 = 0;
    error1 = msg->error;
    currentPos1 = q1 - error1;
}
void Gazebo_Listener::update_jointTwo(const control_msgs::JointControllerState::ConstPtr &msg)
{
    q2 = msg->set_point;
    if (std::abs(q2) < tolerence)
        q2 = 0;
    error2 = msg->error;
    currentPos2 = q2 - error2;
}
void Gazebo_Listener::update_jointThree(const control_msgs::JointControllerState::ConstPtr &msg)
{
    q3 = msg->set_point;
    if (std::abs(q3) < tolerence)
        q3 = 0;
    error3 = msg->error;
    currentPos3 = q3 - error3;
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

    while(ros::ok())
    {
        ros::spinOnce();
        g_listener.outFile << "q1 = " << g_listener.currentPos1 << endl;
        g_listener.outFile << "q2 = " << g_listener.currentPos2 << endl;
        g_listener.outFile << "q3 = " << g_listener.currentPos3 << endl;
        loop_rate.sleep();
    }  

    g_listener.outFile.close();
    ros::spin();
    return 0;
}
