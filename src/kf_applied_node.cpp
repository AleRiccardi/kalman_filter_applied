#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Eigen>

// Main function
int main(int argc, char **argv)
{

    // Launch our ros node
    ros::init(argc, argv, "kf_applied_node");
    ros::NodeHandle nh("~");

    //===================================================================================
    // Wait MODE
    // Wait for a specified amount of time, this is helpfull when you have to attach to
    // the process for debugging with CLion.

    double wait_mode;
    nh.param<double>("wait_mode", wait_mode, 0);
    if (wait_mode > 0)
    {
        ros::Duration wait_dur = ros::Duration(wait_mode);
        ros::Time begin = ros::Time::now();
        ros::Time time = begin;
        ros::Rate rate(5);
        ROS_INFO("Waiting for %f sec", wait_mode);
        while (ros::ok() && time < (begin + wait_dur))
        {
            rate.sleep();
            ros::spinOnce();
            time = ros::Time::now();
        }
        ROS_INFO("End waiting");
    }

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Our  topics (left and right stereo)
    std::string topic_pose;
    nh.param<std::string>("topic_pose", topic_pose, "/kf_applied/pose_noise");
    ROS_INFO("topic pose is: %s", topic_pose.c_str());

    // Location of the ROS bag we want to read in
    std::string path_to_bag;
    nh.param<std::string>("path_bag", path_to_bag, "");
    if (path_to_bag == "")
    {
        ROS_ERROR("No path to ros bag specified");
        return 0;
    }
    ROS_INFO("ros bag path is: %s", path_to_bag.c_str());

    // Get our start location and how much of the bag we want to play
    // Make the bag duration < 0 to just process to the end of the bag
    double bag_start, bag_durr;
    nh.param<double>("bag_start", bag_start, 0);
    nh.param<double>("bag_durr", bag_durr, -1);
    ROS_INFO("bag start: %.1f", bag_start);
    ROS_INFO("bag duration: %.1f", bag_durr);

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Load rosbag here, and find messages we can play
    rosbag::Bag bag;
    bag.open(path_to_bag, rosbag::bagmode::Read);

    // We should load the bag as a view
    // Here we go from beginning of the bag to the end of the bag
    rosbag::View view_full;
    rosbag::View view;

    // Start a few seconds in from the full view time
    // If we have a negative duration then use the full bag length
    view_full.addQuery(bag);
    ros::Time time_init = view_full.getBeginTime();
    time_init += ros::Duration(bag_start);
    ros::Time time_finish = (bag_durr < 0) ? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
    ROS_INFO("time start = %.6f", time_init.toSec());
    ROS_INFO("time end   = %.6f", time_finish.toSec());
    view.addQuery(bag, time_init, time_finish);

    // Check to make sure we have data to play
    if (view.size() == 0)
    {
        ROS_ERROR("No messages to play on specified topics.  Exiting.");
        ros::shutdown();
        return EXIT_FAILURE;
    }

    // Step through the rosbag
    for (const rosbag::MessageInstance &m : view)
    {

        // If ros wants us to stop, break out
        if (!ros::ok())
            break;

        // Handle IMU measurement
        geometry_msgs::PoseStamped::ConstPtr pose_noise = m.instantiate<geometry_msgs::PoseStamped>();
        if (pose_noise != nullptr && m.getTopic() == topic_pose)
        {
            // convert into correct format
            int seq = (*pose_noise).header.seq;
            double timem = (*pose_noise).header.stamp.toSec();
            Eigen::Vector3d pose;
            pose(0) = (*pose_noise).pose.position.x;
            pose(1) = (*pose_noise).pose.position.y;
            pose(2) = (*pose_noise).pose.position.z;

            std::cout << seq << std::endl;
            std::cout << timem << std::endl;
            std::cout << pose << std::endl;
            std::cout << std::endl;
        }
    }

    return 0;
}