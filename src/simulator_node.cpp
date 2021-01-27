#include <ros/ros.h>

#include <cstdio>
#include <iostream>

#include "simulator/SimManager.h"
#include "utils/ParamsManager.h"

int main(int argc, char** argv) {
  try {
    ros::init(argc, argv, "simulator_node");
    ros::NodeHandle nh("~");
    ros::Time::init();

    auto* params = new ParamsManager(nh);
    auto* sim = new SimManager(nh, params);
    ros::Duration dur(0.1);  // faster than the measurements

    while (ros::ok()) {
      sim->user_control();
      sim->propagate();
      sim->publisher();

      ros::spinOnce();
      dur.sleep();
    }

    sim->kill();

  } catch (std::exception& e) {
    std::cout << "An exception occurred: \n" << e.what() << std::endl;
  }

  return 0;
}