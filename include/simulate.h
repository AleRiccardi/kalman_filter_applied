#include <iostream>
#include <time.h>
#include <vector>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

struct arrow
{
    int forward = 105;
    int left = 106;
    int backward = 107;
    int right = 108;
    int up = 117;
    int down = 111;
};

class Simulate
{
public:
    Simulate(ros::NodeHandle nh);
    ~Simulate();

    bool get_acc();
    void propagate();
    void pub_state();

    double _dt = 0.2;

private:
    const double ACC = 0.05;
    Eigen::Matrix<double, 6, 6> _F;
    Eigen::Vector3d _gps_noise;

    int _poses_count = 0;
    Eigen::Matrix<double, 6, 1> _state;
    Eigen::Matrix<double, 6, 1> _state_noise;
    Eigen::Matrix<double, 3, 1> _acc;

    ros::Publisher _pub_pose, _pub_path, _pub_pose_noise, _pub_path_noise;
    std::vector<geometry_msgs::PoseStamped> _poses;
    std::vector<geometry_msgs::PoseStamped> _poses_noise;

    void apply_noise();
};
