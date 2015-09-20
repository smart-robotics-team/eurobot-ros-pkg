// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <beacon_robot_pose_estimate/beacon_robot_pose_estimateConfig.h>

// ROS message includes
#include <sensor_msgs/PointCloud2.h>

// other includes
#include <beacon_robot_pose_estimate_common.cpp>


class beacon_robot_pose_estimate_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<beacon_robot_pose_estimate::beacon_robot_pose_estimateConfig> server;
    dynamic_reconfigure::Server<beacon_robot_pose_estimate::beacon_robot_pose_estimateConfig>::CallbackType f;

    ros::Subscriber input_cloud_;

    beacon_robot_pose_estimate_data component_data_;
    beacon_robot_pose_estimate_config component_config_;
    beacon_robot_pose_estimate_impl component_implementation_;

    beacon_robot_pose_estimate_ros() : np_("~")
    {
        f = boost::bind(&beacon_robot_pose_estimate_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        input_cloud_ = n_.subscribe("input_cloud", 1, &beacon_robot_pose_estimate_impl::topicCallback_input_cloud, &component_implementation_);

        np_.param("x_robot1", component_config_.x_robot1, (double)0.0);
        np_.param("y_robot1", component_config_.y_robot1, (double)0.0);
        np_.param("x_robot2", component_config_.x_robot2, (double)0.0);
        np_.param("y_robot2", component_config_.y_robot2, (double)0.0);
        np_.param("world_link", component_config_.world_link, (std::string)"world");
        np_.param("robot1_link", component_config_.robot1_link, (std::string)"robot1_base_link");
        np_.param("robot2_link", component_config_.robot2_link, (std::string)"robot2_base_link");
    }
    void topicCallback_input_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        component_data_.in_input_cloud = *msg;
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
        configure();
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
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
