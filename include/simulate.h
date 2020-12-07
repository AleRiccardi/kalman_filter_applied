#include <iostream>
#include <time.h>
#include <vector>
#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

struct Character
{
    char value;
    std::string info;
};

class Command
{
public:
    static void print_cmd();

    static Character forward;
    static Character backward;
    static Character left;
    static Character right;
    static Character up;
    static Character down;
    static Character record;
};

class Simulate
{
public:
    Simulate(ros::NodeHandle nh);
    ~Simulate();

    bool user_control();
    void propagate();
    void pub_state();
    void start_record();
    void stop_record();
    void kill();

    double _dt = 0.2;

private:
    void apply_noise();

    rosbag::Bag _bag;

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

    bool _recording;
};
