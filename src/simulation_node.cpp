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
    sim->UserControl();
    sim->propagate();
    sim->PubState();

    ros::spinOnce();
    dur.sleep();
  }

  sim->Kill();

  printf("\nDone.\n");
  return 0;
}
