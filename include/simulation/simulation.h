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

  bool user_control();
  void propagate();
  void pub_state();
  void start_record();
  void stop_record();
  void kill();

  double dt = 0.2;

 private:
  void apply_noise();

  rosbag::Bag bag_;

  const double ACC = 0.05;
  Eigen::Matrix<double, 6, 6> _F;
  Eigen::Vector3d noise_gps_;

  int poses_count_ = 0;
  Eigen::Matrix<double, 6, 1> state_;
  Eigen::Matrix<double, 6, 1> state_gps_;
  Eigen::Matrix<double, 3, 1> acc_;

  ros::Publisher pub_gt_pose_, pub_gt_path_, pub_gps_pose_, pub_gps_path_;
  std::vector<geometry_msgs::PoseStamped> poses_gps_;
  std::vector<geometry_msgs::PoseStamped> poses_noise_;

  bool recording_;
};
