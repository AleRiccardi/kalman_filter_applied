#include <ros/ros.h>

#include <cstdio>
#include <iostream>

#include "simulation/SimManager.h"
#include "utils/ParamsManager.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulation_node");
  ros::NodeHandle nh("~");
  ros::Time::init();

  auto* params = new ParamsManager(nh);
  auto* sim = new SimManager(nh, params);
  ros::Duration dur(0.1);  // faster than the measurements

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
