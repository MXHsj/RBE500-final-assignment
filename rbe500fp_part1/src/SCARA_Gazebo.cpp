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
#define tolerence 0.0001

using namespace std;

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

    for (int i = 0; i < 3; i ++)
    {
        ros::spinOnce();
        // cout << "q1: " << g_listener.q1 << endl;
        // cout << "q2: " << g_listener.q2 << endl;
        // cout << "q3: " << g_listener.q3 << endl;
        loop_rate.sleep();
    }  

    FKsrv.request.q1 = g_listener.q1;
    FKsrv.request.q2 = g_listener.q2;
    FKsrv.request.q3 = g_listener.q3;

    cout << "FKsrv.request.q1: " << FKsrv.request.q1 << endl;
    cout << "FKsrv.request.q2: " << FKsrv.request.q2 << endl;
    cout << "FKsrv.request.q2: " << FKsrv.request.q3 << endl;

    if (FKclient.call(FKsrv))
    {
        ROS_INFO("foward kinematics: \n T = ");
        for(int i = 0; i < 6; i ++)
            cout << (float)FKsrv.response.T[i] << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service calcFK");
        return 1;
    }
    
    double FK_x = (double)FKsrv.response.T[0];
    double FK_y = (double)FKsrv.response.T[1];
    double FK_z = (double)FKsrv.response.T[2];
    double FK_roll = (double)FKsrv.response.T[3];
    double FK_pitch = (double)FKsrv.response.T[4];
    double FK_yaw = (double)FKsrv.response.T[5];

    cout << "FKsrv.response.x: " << FK_x << endl;
    cout << "FKsrv.response.y: " << FK_y << endl;
    cout << "FKsrv.response.z: " << FK_z << endl;

    IKsrv.request.x = FK_x; 
    IKsrv.request.y = FK_y;
    IKsrv.request.z = FK_z;
    IKsrv.request.roll = FK_roll;
    IKsrv.request.pitch = FK_pitch;
    IKsrv.request.yaw = FK_yaw;

    if (IKclient.call(IKsrv))
    {
        ROS_INFO("inverse kinematics: \n");
        cout << "elbow_down solution:" << endl;
        cout << "q1 = " << IKsrv.response.q1[0]*rad2deg << " [deg]" << endl;
        cout << "q2 = " << IKsrv.response.q2[0]*rad2deg << " [deg]" << endl;
        cout << "q3 = " << IKsrv.response.q3[0] << " [m]" << endl;

        cout << "\nelbow-up solution:" << endl;
        cout << "q1 = " << IKsrv.response.q1[1]*rad2deg << " [deg]" << endl;
        cout << "q2 = " << IKsrv.response.q2[1]*rad2deg << " [deg]" << endl;
        cout << "q3 = " << IKsrv.response.q3[1] << " [m]" << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service calcIK");
        return 1;
    }

    ros::spin();
    return 0;
}