#include "ros/ros.h"
#include "rbe500fp_part3/FwdVelKin.h"
#include "rbe500fp_part3/InvVelKin.h"
#include <cmath>

# define L1 0.2		//m
# define L2 0.2		//m
# define L3 0.2		//m
# define L4 0.1 	//m

// calculate forward kinematics (joint velocities -> cartesian velocities)
bool fwdkin(rbe500fp_part3::FwdVelKin::Request &req, rbe500fp_part3::FwdVelKin::Response &res)
{

	// Response matrix is 6x1
	res.TipVel.resize(5);

	// vx
	res.TipVel[0] = -req.dq2*((cos(req.q1)*sin(req.q2))/5 + (cos(req.q2)*sin(req.q1))/5) - req.dq1*(sin(req.q1)/5 + (cos(req.q1)*sin(req.q2))/5 + (cos(req.q2)*sin(req.q1))/5);
	// vy
	res.TipVel[1] = req.dq2*((cos(req.q1)*cos(req.q2))/5 - (sin(req.q1)*sin(req.q2))/5) + req.dq1*(cos(req.q1)/5 + (cos(req.q1)*cos(req.q2))/5 - (sin(req.q1)*sin(req.q2))/5);
	// vz
	res.TipVel[2] = req.dq3;
	// wx
	res.TipVel[3] = 0;
	// wy
	res.TipVel[4] = 0;
	// wz
	res.TipVel[5] = req.dq1 + req.dq2;

	float tolerence = 0.001;

	ROS_INFO("request: dq1=%f, dq2=%f, dq3=%f, q1=%f, q2=%f, q3=%f", (float)req.dq1, (float)req.dq2, (float)req.dq3, (float)req.q1, (float)req.q2, (float)req.q3);
	ROS_INFO("sending back response TipVel:");

	for(int i = 0; i < 6; i ++)
	{
		if(res.TipVel[i] <= tolerence)
			res.TipVel[i] = 0;

		std::cout << (float)res.TipVel[i] << "\t";
	}

	return true;
}

// calculate inverse kinematics (cartestian velocities -> joint velocities)
bool invkin(rbe500fp_part3::InvVelKin::Request &req, rbe500fp_part3::InvVelKin::Response &res)
{
	float tolerence = 0.001;
	/*
	// velocity joint 1
	// res.dq1 = (50*req.daz*cos(req.q2) - 255*req.dy*cos(req.q1) + 255*req.dx*sin(req.q1) + 5*req.dy*cos(req.q1 + 2*req.q2) - 5*req.dx*sin(req.q1 + 2*req.q2))/(cos(2*req.q2) - 51);
	res.dq1 = req.dz*sin(req.q1 + 1.57) - req.dy*cos(req.q1 + 1.57)*(L3*sin(req.q2 + req.q3)) + L2*sin(req.q2) - req.dx*cos(req.q1)*(L3*cos(req.q2 + req.q3)) + L2*cos(req.q2);
	if (res.dq1 < tolerence)
	{
		res.dq1 = 0;
	}

	// velocity joint 2
	// res.dq2 = (130*req.dy*cos(req.q1) - 25*req.daz*cos(req.q2) - 25*req.daz - 130*req.dx*sin(req.q1) + 5*req.dx*cos(req.q1)*sin(req.q2) + 5*req.dy*sin(req.q1)*sin(req.q2) - 5*req.dy*cos(req.q1)*pow(cos(req.q2),2) + 5*req.dx*pow(cos(req.q2),2)*sin(req.q1) + 5*req.dx*cos(req.q1)*cos(req.q2)*sin(req.q2) + 5*req.dy*cos(req.q2)*sin(req.q1)*sin(req.q2))/(pow(cos(req.q2),2) - 26);
	res.dq2 = - req.dz*cos(req.q1 + 1.57) - req.dy*sin(req.q1 + 1.57)*(L3*sin(req.q2 + req.q3)) + L2*sin(req.q2) - req.dx*sin(req.q1)*(L3*cos(req.q2 + req.q3)) + L2*cos(req.q2);
 
	if (res.dq2 < tolerence)
	{
		res.dq2 = 0;
	}

	// velocity joint 3
	res.dq3 = req.dz;
	if (res.dq3 < tolerence)
	{
		res.dq3 = 0;
	}
	*/

	float a = (-L1*sin(req.q1))-(L2*sin(req.q1 + req.q2));
	float b = -L2*sin(req.q1 + req.q2);
	float c = 0;
	float d = (L1*cos(req.q1)) + (L2*cos(req.q1+req.q2));
	float e = L2*cos(req.q1 + req.q2);
	float f = 0;
	float g = 0;
	float h = 0;
	float i = -1;


	float determinate = (a*((e*i)-(f*h))) - (b*((d*i)-(f*g))) + (c*((d*h)-(e*g)));

	if (determinate == 0)
	{
		std::cout << "There is no inverse" << std::endl;
	}
	else
	{
		float at = (-L1*sin(req.q1))-(L2*sin(req.q1 + req.q2)); 
		float bt = (L1*cos(req.q1)) + (L2*cos(req.q1+req.q2));
		float ct = 0;											
		float dt = -L2*sin(req.q1 + req.q2);
		float et = L2*cos(req.q1 + req.q2);						
		float ft = 0;
		float gt = 0;
		float ht = 0;
		float it = -1;											

		float adj_11 = (et*it) - (ft*ht);
		float adj_12 = -((dt*it) - (ft*gt));
		float adj_13 = (dt*ht) - (et*gt);
		float adj_21 = -((bt*it) - (ct*ht));
		float adj_22 = (at*it) - (ct*gt);
		float adj_23 = -((at*ht) - (bt*gt));
		float adj_31 = (bt*ft) - (ct*et);
		float adj_32 = -((at*ft) - (ct*dt));
		float adj_33 = (at*et) - (bt*dt);

		adj_11 = adj_11 / determinate;
		adj_12 = adj_12 / determinate;
		adj_13 = adj_13;
		adj_21 = adj_21 / determinate;
		adj_22 = adj_22 / determinate;
		adj_23 = adj_23;
		adj_31 = adj_31;
		adj_32 = adj_32;
		adj_33 = adj_33 / determinate;
		
		res.dq1 = (adj_11*req.dx) + (adj_12*req.dy) + (adj_13* req.dz);
		res.dq2 = (adj_21*req.dx) + (adj_22*req.dy) + (adj_23* req.dz);
		res.dq3 = (adj_31*req.dx) + (adj_32*req.dy) + (adj_33* req.dz);

	}


	ROS_INFO("request: dx=%f, dy=%f, dz=%f, dax=%f, day=%f, daz=%f, q1=%f, q2=%f, q3=%f", (float)req.dx, (float)req.dy, (float)req.dz, (float)req.dax, (float)req.day, (float)req.daz, (float)req.q1, (float)req.q2, (float)req.q3);
	ROS_INFO("sending back response: dq1 = %f", "dq2 = %f", "dq3 = %f", (float)res.dq1, (float)res.dq2, (float)res.dq3);

	std::cout << (float)res.dq1 << "\t";
	std::cout << (float)res.dq2 << "\t";
	std::cout << (float)res.dq3 << "\t";

	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "VelocityKin_server");
  ros::NodeHandle n;

  // Advertise a service for forward kinematics
  ros::ServiceServer service = n.advertiseService("calculate_FK", fwdkin);

  // Advertise a service for inverse kinematics
  ros::ServiceServer servive = n.advertiseService("calculate_IK", invkin);

  ROS_INFO("Ready to calculate forward velocity kinematics for SCARA robot");
  ROS_INFO("Ready to calculate inverse velocity kinematics for SCARA robot");

  ros::spin();

  return 0;
}

