/*
 * VelocityCmd.cpp
 *
 *  Created on: Dec 10, 2019
 *     
 */

#include "ros/ros.h"
#include "rbe500fp_part3/RefVel.h"
#include "control_msgs/JointControllerState.h"
#include <cstdlib>

#define rad2deg 180/3.1415
#define tolerence 0.0001

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Velocity_Client");

    // Make sure an argument was given
    if (argc != 4)
    {
        ROS_INFO("Usage: Input a Reference Velocity in Cartestian Coordinates");
        return 1;
    }
    ros::NodeHandle n;

    // Create a client relationship to the VelServer
    ros::ServiceClient RefClient = n.serviceClient<rbe500fp_part3::RefVel>("VelServer");

    rbe500fp_part3::RefVel RefSrv;

    // Parse the argument
    RefSrv.request.refx = atof(argv[1]);
    RefSrv.request.refy = atof(argv[2]);
    RefSrv.request.refz = atof(argv[3]);


    // Send the reference position to the VelServer
    if (RefClient.call(RefSrv))
    {
        ROS_INFO("Moving: \n Motion = ");
        if (RefSrv.response.done)
        {
            ROS_INFO("Complete");
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

    ros::spin();
    return 0;
}
