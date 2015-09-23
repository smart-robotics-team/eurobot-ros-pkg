// ROS message includes
#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud2.h>

/* protected region user include files on begin */
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <stdlib.h>

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>                                         // for in-/output
#include <string.h>                                        // strcat

//Include system headers
#include <cstring>
#include <iostream>
#include <cstdio>

#include <sstream>
#include <math.h>
#include <vector>
#include <list>


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
    double detection_distance;
};

class beacon_robot_pose_estimate_data
{
// autogenerated: don't touch this class
public:
    //input data
    sensor_msgs::PointCloud2 in_input_cloud;
    //output data
    geometry_msgs::Pose2D out_robot1_pose;
    bool out_robot1_pose_active;
    geometry_msgs::Pose2D out_robot2_pose;
    bool out_robot2_pose_active;
};

class beacon_robot_pose_estimate_impl
{
    /* protected region user member variables on begin */
	beacon_robot_pose_estimate_config localconfig;

	tf::StampedTransform t;
	tf::TransformBroadcaster broadcaster;

	std::list<geometry_msgs::PoseStamped> robots;

	bool robot1_output_ready;
	bool robot2_output_ready;
    /* protected region user member variables end */

public:
    beacon_robot_pose_estimate_impl() 
    {
        /* protected region user constructor on begin */
    	robot1_output_ready = false;
    	robot2_output_ready = false;

        geometry_msgs::PoseStamped tmp;
        tmp.header.frame_id = "0";
        tmp.pose.position.x = 0.0;
        tmp.pose.position.y = 0.0;
        tmp.pose.position.z = 0.0;
        tmp.pose.orientation.x = 0.0;
        tmp.pose.orientation.y = 0.0;
        tmp.pose.orientation.z = 1.0;
        tmp.pose.orientation.w = 0.0;
        robots.push_back(tmp);
        tmp.header.frame_id = "1";
        tmp.pose.position.x = 0.0;
        tmp.pose.position.y = 0.0;
        tmp.pose.position.z = 0.0;
        tmp.pose.orientation.x = 0.0;
        tmp.pose.orientation.y = 0.0;
        tmp.pose.orientation.z = 1.0;
        tmp.pose.orientation.w = 0.0;
        robots.push_back(tmp);

    	/* protected region user constructor end */
    }

    void configure(beacon_robot_pose_estimate_config config) 
    {
        /* protected region user configure on begin */
    	localconfig = config;
    	//robots. .at(0).pose.position.x = localconfig.x_robot1;
    	//robots.at(0).pose.position.y = localconfig.y_robot1;
    	//robots.at(1).pose.position.x = localconfig.x_robot2;
    	//robots.at(1).pose.position.y = localconfig.y_robot2;
        /* protected region user configure end */
    }

    void update(beacon_robot_pose_estimate_data &data, beacon_robot_pose_estimate_config config)
    {
        /* protected region user update on begin */
    	data.out_robot1_pose_active = robot1_output_ready;
    	if(robot1_output_ready)
    	{
    		//data.out_robot1_pose.x = robots.at(0).pose.position.x;
    		robot1_output_ready = false;
    	}
    	else
    	{

    	}

    	data.out_robot2_pose_active = robot2_output_ready;
    	if(robot2_output_ready)
		{
			robot2_output_ready = false;
		}
		else
		{

		}
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
