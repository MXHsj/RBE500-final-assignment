/*
 * toFile.cpp
 *
 *  Created on: Nov 25, 2019
 *     
 */

#include "ros/ros.h"
#include "rbe500fp_part2/calcIK.h"
#include "rbe500fp_part2/calcFK.h"
#include "sensor_msgs/JointState.h"
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

    ofstream outFile;
    void update_joints(const sensor_msgs::JointState::ConstPtr &msg);


    Gazebo_Listener()
    {
        outFile.open("/home/ellie/catkin_ws/src/RBE500-final-assignment/rbe500fp_part2/src/output.txt");
    }
};

void Gazebo_Listener::update_joints(const sensor_msgs::JointState::ConstPtr &msg)
{
    q1 = msg->position[0];
    if (std::abs(q1) < tolerence)
        q1 = 0;
    

    q2 = msg->position[1];
    if (std::abs(q2) < tolerence)
        q2 = 0;

    q3 = msg->position[2];
    if (std::abs(q3) < tolerence)
        q3 = 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SCARA_Gazebo_client");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);
    Gazebo_Listener g_listener;

    ros::Subscriber jointOne = n.subscribe("/custom_scara/joint_states", 1, &Gazebo_Listener::update_joints, &g_listener);
    
    while(ros::ok())
    {
        ros::spinOnce();
        // g_listener.outFile << "q1 = " << g_listener.currentPos1 << endl;
        // g_listener.outFile << "q2 = " << g_listener.currentPos2 << endl;
        g_listener.outFile << "q3," << g_listener.q3 << endl;
        loop_rate.sleep();
    }  

    g_listener.outFile.close();
    ros::spin();
    return 0;
}
