// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <beacon_pose_init/beacon_pose_initConfig.h>

// ROS message includes
#include <sensor_msgs/PointCloud2.h>

// other includes
#include <beacon_pose_init_common.cpp>


class beacon_pose_init_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<beacon_pose_init::beacon_pose_initConfig> server;
    dynamic_reconfigure::Server<beacon_pose_init::beacon_pose_initConfig>::CallbackType f;

    ros::Subscriber input_;

    beacon_pose_init_data component_data_;
    beacon_pose_init_config component_config_;
    beacon_pose_init_impl component_implementation_;

    beacon_pose_init_ros() : np_("~")
    {
        f = boost::bind(&beacon_pose_init_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        input_ = n_.subscribe("input", 1, &beacon_pose_init_impl::topicCallback_input, &component_implementation_);

        np_.param("x_object1", component_config_.x_object1, (double)0.0);
        np_.param("y_object1", component_config_.y_object1, (double)0.4);
        np_.param("x_object2", component_config_.x_object2, (double)0.0);
        np_.param("y_object2", component_config_.y_object2, (double)1.6);
        np_.param("x_init_pose", component_config_.x_init_pose, (double)0.0);
        np_.param("y_init_pose", component_config_.y_init_pose, (double)0.0);
        np_.param("base_beacon_link", component_config_.base_beacon_link, (std::string)"beacon_link");
        np_.param("laser_beacon_link", component_config_.laser_beacon_link, (std::string)"beacon_laser_link");
    }
    void topicCallback_input(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        component_data_.in_input = *msg;
    }

    void configure_callback(beacon_pose_init::beacon_pose_initConfig &config, uint32_t level)
    {
        component_config_.x_object1 = config.x_object1;
        component_config_.y_object1 = config.y_object1;
        component_config_.x_object2 = config.x_object2;
        component_config_.y_object2 = config.y_object2;
        component_config_.x_init_pose = config.x_init_pose;
        component_config_.y_init_pose = config.y_init_pose;
        component_config_.base_beacon_link = config.base_beacon_link;
        component_config_.laser_beacon_link = config.laser_beacon_link;
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

    ros::init(argc, argv, "beacon_pose_init");

    beacon_pose_init_ros node;
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
