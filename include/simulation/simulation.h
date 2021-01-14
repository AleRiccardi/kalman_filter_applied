#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <time.h>

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

struct Character {
  char value;
  std::string info;
};

class Command {
 public:
  static void print_cmd();

  static Character forward_;
  static Character backward_;
  static Character left_;
  static Character right_;
  static Character up_;
  static Character down_;
  static Character record_;
};

class Simulation {
 public:
  Simulation(ros::NodeHandle nh);
  ~Simulation() = default;

  bool UserControl();
  void propagate();
  void PubState();
  void StartRecord();
  void StopRecord();
  void Kill();

  double dt = 0.2;

 private:
  void apply_noise();

  rosbag::Bag bag_;

  const double ACC = 0.05;
  Eigen::Matrix<double, 6, 6> _F;
  Eigen::Vector3d gps_noise_;

  int poses_count_ = 0;
  Eigen::Matrix<double, 6, 1> state_;
  Eigen::Matrix<double, 6, 1> state_noise_;
  Eigen::Matrix<double, 3, 1> acc_;

  ros::Publisher pub_pose_gt_, pub_path_gt_, pub_pose_noise_, pub_path_noise_;
  std::vector<geometry_msgs::PoseStamped> poses_;
  std::vector<geometry_msgs::PoseStamped> poses_noise_;

  bool recording_;
};
