// ROS message includes
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

/* protected region user include files on begin */
/* protected region user include files end */

class beacon_robot_pose_estimate_config
{
public:
    double x_robot1;
    double y_robot1;
    double x_robot2;
    double y_robot2;
    std::string world_link;
    std::string robot1_link;
    std::string robot2_link;
};

class beacon_robot_pose_estimate_data
{
// autogenerated: don't touch this class
public:
    //input data
    sensor_msgs::PointCloud2 in_input_cloud;
    //output data
};

class beacon_robot_pose_estimate_impl
{
    /* protected region user member variables on begin */
    /* protected region user member variables end */

public:
    beacon_robot_pose_estimate_impl() 
    {
        /* protected region user constructor on begin */
        /* protected region user constructor end */
    }

    void configure(beacon_robot_pose_estimate_config config) 
    {
        /* protected region user configure on begin */
        /* protected region user configure end */
    }

    void update(beacon_robot_pose_estimate_data &data, beacon_robot_pose_estimate_config config)
    {
        /* protected region user update on begin */
        /* protected region user update end */
    }

    void topicCallback_input_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        /* protected region user implementation of subscribe callback for input_cloud on begin */
        /* protected region user implementation of subscribe callback for input_cloud end */
    }



    /* protected region user additional functions on begin */
    /* protected region user additional functions end */
};
