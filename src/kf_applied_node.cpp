#include <stdio.h>
#include <iostream>
#include <ros/ros.h>

#include "simulate.h"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "kf_applied_node");
	ros::NodeHandle nh("~");
	ros::Time::init();

	Simulate *sim = new Simulate(nh);
	ros::Duration dur(sim->_dt);

	while (ros::ok())
	{
		sim->get_acc();
		sim->propagate();
		sim->pub_state();

		ros::spinOnce();
		dur.sleep();
	}

	printf("\nDone.\n");
	return 0;
}
