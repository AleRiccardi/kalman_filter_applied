#include <iostream>
#include <ros/ros.h>
#include <stdio.h>

#include "simulation.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "simulation_node");
  ros::NodeHandle nh("~");
  ros::Time::init();

  Simulation *sim = new Simulation(nh);
  ros::Duration dur(sim->_dt);

  while (ros::ok()) {
    sim->user_control();
    sim->propagate();
    sim->pub_state();

    ros::spinOnce();
    dur.sleep();
  }

  sim->kill();

  printf("\nDone.\n");
  return 0;
}
