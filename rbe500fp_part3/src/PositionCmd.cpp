/*
 * PDClient.cpp
 *
 *  Created on: Nov 25, 2019
 *     
 */

#include "ros/ros.h"
#include "rbe500fp_part3/RefPos.h"
#include "control_msgs/JointControllerState.h"
#include <cstdlib>

#define rad2deg 180/3.1415
#define tolerence 0.0001

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PD_Client");

    // Make sure an argument was given
    if (argc != 4)
    {
        ROS_INFO("Usage: Input a Reference Position for Joint1 and Joint2 in [rad], and Joint 3 in [m]");
        return 1;
    }
    ros::NodeHandle n;

    // Create a client relationship to the PD_Server
    ros::ServiceClient RefClient = n.serviceClient<rbe500fp_part3::RefPos>("PD_Server");

    rbe500fp_part3::RefPos RefSrv;

    // Parse the argument
    RefSrv.request.ref1 = atof(argv[1]);
    RefSrv.request.ref2 = atof(argv[2]);
    RefSrv.request.ref3 = atof(argv[3]);


    // Send the reference position to the PD_Service
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
