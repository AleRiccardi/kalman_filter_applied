#include "simulate.h"
#include <conio.h>
#include <Eigen/Dense>

Simulate::Simulate(ros::NodeHandle nh)
{
    // Initialize random seed
    std::srand(time(NULL));

    _state.setZero();
    ROS_INFO_STREAM("Init State: \n"
                    << _state);

    _acc.setZero();
    ROS_INFO_STREAM("Init Acceleration: \n"
                    << _acc);

    _gps_noise = Eigen::Vector3d(0.15, 0.15, 0.05);

    std::cout << _state << std::endl;
    _F << 1, 0, 0, _dt, 0, 0,
        0, 1, 0, 0, _dt, 0,
        0, 0, 1, 0, 0, _dt,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    ROS_INFO_STREAM("State Evolution: \n"
                    << _F);

    _pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/kf_applied/gt_pose", 2);
    ROS_INFO("Publishing: %s", _pub_pose.getTopic().c_str());

    _pub_path = nh.advertise<nav_msgs::Path>("/kf_applied/gt_path", 2);
    ROS_INFO("Publishing: %s", _pub_path.getTopic().c_str());

    _pub_pose_noise = nh.advertise<geometry_msgs::PoseStamped>("/kf_applied/pose_noise", 2);
    ROS_INFO("Publishing: %s", _pub_pose_noise.getTopic().c_str());

    _pub_path_noise = nh.advertise<nav_msgs::Path>("/kf_applied/path_noise", 2);
    ROS_INFO("Publishing: %s", _pub_path_noise.getTopic().c_str());
}

Simulate::~Simulate()
{
}

bool Simulate::get_acc()
{
    _acc = Eigen::Matrix<double, 3, 1>::Zero();

    if (kbhit())
    {
        // Stores the pressed key in ch
        char ch = getch();

        switch (int(ch))
        {
        case arrow().forward:
            _acc(0) = ACC;
            return true;
        case arrow().backward:
            _acc(0) = -ACC;
            return true;
        case arrow().left:
            _acc(1) = -ACC;
            return true;
        case arrow().right:
            _acc(1) = ACC;
            return true;
        case arrow().up:
            _acc(2) = ACC;
            return true;
        case arrow().down:
            _acc(2) = -ACC;
            return true;
        }
    }
    return false;
}

void Simulate::propagate()
{
    // _state evolution
    for (int i = 0; i <= _F.rows() - 1; i++)
    {
        double val = _F.row(i).dot(_state);
        _state(i, 0) = val;
    }

    _state.tail(3) += _acc;

    apply_noise();
}

void Simulate::pub_state()
{
    // Create pose (note we use the bag time)
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.seq = _poses_count;
    pose.header.frame_id = "global";
    pose.pose.position.x = _state(0, 0);
    pose.pose.position.y = _state(1, 0);
    pose.pose.position.z = _state(2, 0);

    _pub_pose.publish(pose);

    // =======================================================

    // Create pose (note we use the bag time)
    geometry_msgs::PoseStamped pose_noise;
    pose_noise.header.stamp = ros::Time::now();
    pose_noise.header.seq = _poses_count;
    pose_noise.header.frame_id = "global";
    pose_noise.pose.position.x = _state_noise(0, 0);
    pose_noise.pose.position.y = _state_noise(1, 0);
    pose_noise.pose.position.z = _state_noise(2, 0);

    _pub_pose_noise.publish(pose_noise);

    //=========================================================
    //=========================================================

    // Append to our pose vectors
    _poses.push_back(pose);
    _poses_noise.push_back(pose_noise);

    // Create our path
    // NOTE: We downsample the number of poses as needed to prevent rviz crashes
    // NOTE: https://github.com/ros-visualization/rviz/issues/1107
    nav_msgs::Path arrIMU;
    arrIMU.header.stamp = pose.header.stamp;
    arrIMU.header.seq = _poses_count;
    arrIMU.header.frame_id = "global";
    for (size_t i = 0; i < _poses.size(); i += std::floor(_poses.size() / 16384.0) + 1)
    {
        arrIMU.poses.push_back(_poses.at(i));
    }
    _pub_path.publish(arrIMU);

    // =======================================================

    nav_msgs::Path arrIMU_noise;
    arrIMU_noise.header.stamp = pose_noise.header.stamp;
    arrIMU_noise.header.seq = _poses_count;
    arrIMU_noise.header.frame_id = "global";
    for (size_t i = 0; i < _poses_noise.size(); i += std::floor(_poses_noise.size() / 16384.0) + 1)
    {
        arrIMU_noise.poses.push_back(_poses_noise.at(i));
    }
    _pub_path_noise.publish(arrIMU_noise);

    // Move forward in time
    _poses_count++;
}

void Simulate::apply_noise()
{
    Eigen::Vector3d rands;
    rands(0,0) = (double)((std::rand() % 200) - 100) / 100;
    rands(1,0) = (double)((std::rand() % 200) - 100) / 100;
    rands(2,0) = (double)((std::rand() % 200) - 100) / 100;
    
    // TODO: inline multiplication
    Eigen::Vector3d noise;
    noise(0) = _gps_noise(0) * rands(0);
    noise(1) = _gps_noise(1) * rands(1);
    noise(2) = _gps_noise(2) * rands(2);

    _state_noise = _state;
    _state_noise.head(3) = _state.head(3) + noise;
}