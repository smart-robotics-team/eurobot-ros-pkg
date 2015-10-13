// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <beacon_robot_pose_estimate/beacon_robot_pose_estimateConfig.h>

// ROS message includes
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>

// other includes
#include <beacon_robot_pose_estimate_common.cpp>


class beacon_robot_pose_estimate_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<beacon_robot_pose_estimate::beacon_robot_pose_estimateConfig> server;
    dynamic_reconfigure::Server<beacon_robot_pose_estimate::beacon_robot_pose_estimateConfig>::CallbackType f;

    ros::Publisher robot1_pose_;
    ros::Publisher robot2_pose_;
    ros::Subscriber input_cloud_;
    ros::Subscriber odom1_;
    ros::Subscriber odom2_;

    beacon_robot_pose_estimate_data component_data_;
    beacon_robot_pose_estimate_config component_config_;
    beacon_robot_pose_estimate_impl component_implementation_;

    beacon_robot_pose_estimate_ros() : np_("~")
    {
        f = boost::bind(&beacon_robot_pose_estimate_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        robot1_pose_ = n_.advertise<geometry_msgs::Pose2D>("robot1_pose", 1);
        robot2_pose_ = n_.advertise<geometry_msgs::Pose2D>("robot2_pose", 1);
        input_cloud_ = n_.subscribe("input_cloud", 1, &beacon_robot_pose_estimate_impl::topicCallback_input_cloud, &component_implementation_);
        odom1_ = n_.subscribe("odom1", 1, &beacon_robot_pose_estimate_impl::topicCallback_odom1, &component_implementation_);
        odom2_ = n_.subscribe("odom2", 1, &beacon_robot_pose_estimate_impl::topicCallback_odom2, &component_implementation_);

        np_.param("x_robot1", component_config_.x_robot1, (double)0.0);
        np_.param("y_robot1", component_config_.y_robot1, (double)0.0);
        np_.param("x_robot2", component_config_.x_robot2, (double)0.0);
        np_.param("y_robot2", component_config_.y_robot2, (double)0.0);
        np_.param("world_link", component_config_.world_link, (std::string)"world");
        np_.param("robot1_link", component_config_.robot1_link, (std::string)"robot1_base_link");
        np_.param("robot2_link", component_config_.robot2_link, (std::string)"robot2_base_link");
        np_.param("detection_distance", component_config_.detection_distance, (double)0.07);
    }
    void topicCallback_input_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        component_data_.in_input_cloud = *msg;
    }
    void topicCallback_odom1(const nav_msgs::Odometry::ConstPtr& msg)
    {
        component_data_.in_odom1 = *msg;
    }
    void topicCallback_odom2(const nav_msgs::Odometry::ConstPtr& msg)
    {
        component_data_.in_odom2 = *msg;
    }

    void configure_callback(beacon_robot_pose_estimate::beacon_robot_pose_estimateConfig &config, uint32_t level)
    {
        component_config_.x_robot1 = config.x_robot1;
        component_config_.y_robot1 = config.y_robot1;
        component_config_.x_robot2 = config.x_robot2;
        component_config_.y_robot2 = config.y_robot2;
        component_config_.world_link = config.world_link;
        component_config_.robot1_link = config.robot1_link;
        component_config_.robot2_link = config.robot2_link;
        component_config_.detection_distance = config.detection_distance;
        configure();
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_robot1_pose_active = true;
        component_data_.out_robot2_pose_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_robot1_pose_active)
            robot1_pose_.publish(component_data_.out_robot1_pose);
        if (component_data_.out_robot2_pose_active)
            robot2_pose_.publish(component_data_.out_robot2_pose);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "beacon_robot_pose_estimate");

    beacon_robot_pose_estimate_ros node;
    node.configure();

    ros::Rate loop_rate(50.0);

    while(node.n_.ok())
    {
        node.update();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
