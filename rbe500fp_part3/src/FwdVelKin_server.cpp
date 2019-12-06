#include "ros/ros.h"

int main()
{
    return 0;
}
/*
#include "xihan_rbe/calcFK.h"
#include "xihan_rbe/calcJac.h"
#include <cmath>

# define L1 0.2		//m
# define L2 0.2		//m
# define L3 0.2		//m
# define L4 0.1 	//m

// calculate forward kinematics
bool fwdkin(xihan_rbe::calcFK::Request &req, xihan_rbe::calcFK::Response &res)
{
	res.T.resize(16);
	float tolerence = 0.001;

	res.T[0] = cos(req.q1 + req.q2);
	res.T[1] = sin(req.q1 + req.q2);
	res.T[3] = L2*cos(req.q1) + L3*cos(req.q1 + req.q2);
	res.T[4] = sin(req.q1 + req.q2);
	res.T[5] = -cos(req.q1 + req.q2);
	res.T[7] = L2*sin(req.q1) + L3*sin(req.q1 + req.q2);
	res.T[10] = -1;
	res.T[11] = L1 - L4 - req.q3;
	res.T[15] = 1;

	ROS_INFO("request: q1=%f, q2=%f, q3=%f", (float)req.q1, (float)req.q2, (float)req.q3);
	ROS_INFO("sending back response T:");

	int j = 0;
	for(int i = 0; i < 16; i ++)
	{
		if(res.T[i] <= tolerence)
			res.T[i] = 0;

		std::cout << (float)res.T[i] << "\t";

		if(j == 3)		// display in 4x4 format
		{
			std::cout << std::endl;
			j = 0;
		}
		else
			j++;
	}

	return true;
}

// calculate Jacobian
bool jacobians(xihan_rbe::calcJac::Request &req, xihan_rbe::calcJac::Response &res)
{
	res.Jac.resize(18);
	float tolerence = 0.001;

	res.Jac[0] = -(L2*sin(req.q1)+L3*sin(req.q1+req.q2));
	res.Jac[1] = -L2*sin(req.q1+req.q2);
	res.Jac[3] = (L2*cos(req.q1)+L3*cos(req.q1+req.q2));
	res.Jac[4] = L2*cos(req.q1+req.q2);
	res.Jac[8] = 1;
	res.Jac[15] = 1;
	res.Jac[16] = 1;

	ROS_INFO("request: q1=%f, q2=%f, q3=%f", (float)req.q1, (float)req.q2, (float)req.q3);
	ROS_INFO("sending back response J:");

	int j = 0;
	for(int i = 0; i < 18; i ++)
	{
		if(res.Jac[i] <= tolerence)
			res.Jac[i] = 0;

		std::cout << (float)res.Jac[i] << "\t";

		if(j == 2)		// display in 6x3 format
		{
			std::cout << std::endl;
			j = 0;
		}
		else
			j++;
	}

	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SCARA_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("calculate_FK", fwdkin);
  ros::ServiceServer servive = n.advertiseService("calculate_Jac", jacobians);

  ROS_INFO("Ready to calculate forward kinematics for SCARA robot");
  ROS_INFO("Ready to calculate Jacobian for SCARA robot");

  ros::spin();

  return 0;
}

*/