#include <ros/ros.h>

#include <cstdio>
#include <iostream>

#include "simulation/simulation.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulation_node");
  ros::NodeHandle nh("~");
  ros::Time::init();

  auto* sim = new Simulation(nh);
  ros::Duration dur(sim->dt);

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
