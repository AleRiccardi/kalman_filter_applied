#include "simulate.h"

#include <ros/package.h>
#include <conio.h>
#include <string>

Character Command::forward = {'i', "Forward"};
Character Command::backward = {'k', "Backward"};
Character Command::left = {'j', "Left"};
Character Command::right = {'l', "Right"};
Character Command::up = {'u', "Up"};
Character Command::down = {'o', "Down"};
Character Command::record = {'r', "Record"};

void Command::print_cmd()
{
    std::cout << std::endl;
    ROS_INFO("Use the following commands for moving the agent: ");
    ROS_INFO("%c - %s", Command::forward.value, Command::forward.info.c_str());
    ROS_INFO("%c - %s", Command::backward.value, Command::backward.info.c_str());
    ROS_INFO("%c - %s", Command::left.value, Command::left.info.c_str());
    ROS_INFO("%c - %s", Command::right.value, Command::right.info.c_str());
    ROS_INFO("%c - %s", Command::up.value, Command::up.info.c_str());
    ROS_INFO("%c - %s", Command::down.value, Command::down.info.c_str());
    ROS_INFO("%c - %s", Command::record.value, Command::record.info.c_str());
    std::cout << std::endl;
}

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
    _F << 1, 0, 0, _dt, 0, 0, 0, 1, 0, 0, _dt, 0, 0, 0, 1, 0, 0, _dt, 0, 0, 0, 1,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;

    ROS_INFO_STREAM("State Evolution: \n"
                    << _F);

    _pub_pose =
        nh.advertise<geometry_msgs::PoseStamped>("/kf_applied/gt_pose", 2);
    ROS_INFO("Publishing: %s", _pub_pose.getTopic().c_str());

    _pub_path = nh.advertise<nav_msgs::Path>("/kf_applied/gt_path", 2);
    ROS_INFO("Publishing: %s", _pub_path.getTopic().c_str());

    _pub_pose_noise =
        nh.advertise<geometry_msgs::PoseStamped>("/kf_applied/pose_noise", 2);
    ROS_INFO("Publishing: %s", _pub_pose_noise.getTopic().c_str());

    _pub_path_noise = nh.advertise<nav_msgs::Path>("/kf_applied/path_noise", 2);
    ROS_INFO("Publishing: %s", _pub_path_noise.getTopic().c_str());

    Command::print_cmd();
}

Simulate::~Simulate() {}

bool Simulate::user_control()
{
    _acc = Eigen::Matrix<double, 3, 1>::Zero();

    if (kbhit())
    {
        bool known_cmd = true;
        // Stores the pressed key in ch
        char u_input = int(getch());

        if (u_input == Command::forward.value)
        {
            _acc(0) = ACC;
        }
        else if (u_input == Command::backward.value)
        {
            _acc(0) = -ACC;
        }
        else if (u_input == Command::left.value)
        {
            _acc(1) = ACC;
        }
        else if (u_input == Command::right.value)
        {
            _acc(1) = -ACC;
        }
        else if (u_input == Command::up.value)
        {
            _acc(2) = ACC;
        }
        else if (u_input == Command::down.value)
        {
            _acc(2) = -ACC;
        }
        else if (u_input == Command::record.value)
        {
            if (_recording == false)
            {
                start_record();
            } else {
                stop_record();
            }
        }
        else
        {
            known_cmd = false;
        }
        return known_cmd;
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
    for (size_t i = 0; i < _poses.size();
         i += std::floor(_poses.size() / 16384.0) + 1)
    {
        arrIMU.poses.push_back(_poses.at(i));
    }
    _pub_path.publish(arrIMU);

    // =======================================================

    nav_msgs::Path arrIMU_noise;
    arrIMU_noise.header.stamp = pose_noise.header.stamp;
    arrIMU_noise.header.seq = _poses_count;
    arrIMU_noise.header.frame_id = "global";
    for (size_t i = 0; i < _poses_noise.size();
         i += std::floor(_poses_noise.size() / 16384.0) + 1)
    {
        arrIMU_noise.poses.push_back(_poses_noise.at(i));
    }
    _pub_path_noise.publish(arrIMU_noise);

    // Move forward in time
    _poses_count++;

    if (_recording == true)
    {
        _bag.write(_pub_pose.getTopic().c_str(), pose.header.stamp, pose);
    }
}

void Simulate::apply_noise()
{
    Eigen::Vector3d rands;
    rands(0, 0) = (double)((std::rand() % 200) - 100) / 100;
    rands(1, 0) = (double)((std::rand() % 200) - 100) / 100;
    rands(2, 0) = (double)((std::rand() % 200) - 100) / 100;

    // TODO: inline multiplication
    Eigen::Vector3d noise;
    noise(0) = _gps_noise(0) * rands(0);
    noise(1) = _gps_noise(1) * rands(1);
    noise(2) = _gps_noise(2) * rands(2);

    _state_noise = _state;
    _state_noise.head(3) = _state.head(3) + noise;
}

void Simulate::start_record()
{
    _recording = true;
    // TODO: get name from ros
    std::string path_pkg = ros::package::getPath("kf_applied") + "/data/";
    std::string file_name = std::to_string((int)ros::Time::now().toSec()) + ".bag";
    std::string pathPname = path_pkg + file_name;
    ROS_INFO("Recording started");
    _bag.open(pathPname, rosbag::bagmode::Write);
}

void Simulate::stop_record()
{
    ROS_INFO("Recording stopped");
    _recording = false;
    _bag.close();
}

void Simulate::kill()
{
    if (_recording == true)
    {
        _recording = false;
        _bag.close();
    }
}