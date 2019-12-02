/*
 * PDServer.cpp
 *
 *  Created on: Nov 21, 2019
 *     
 */

#include "ros/ros.h"
#include "rbe500fp_part2/RefPos.h"
#include "control_msgs/JointControllerState.h"
#include <cstdlib>

#define rad2deg 180/3.1415
#define tolerence 0.0001

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PD_Client");

    if (argc != 2)
    {
        ROS_INFO("Usage: Input a Reference Position for Joint 3 in [m]");
        return 1;
    }
    ros::NodeHandle n;

    ros::ServiceClient RefClient = n.serviceClient<rbe500fp_part2::RefPos>("PD_Server");

    rbe500fp_part2::RefPos RefSrv;

    RefSrv.request.ref = atof(argv[1]);

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
