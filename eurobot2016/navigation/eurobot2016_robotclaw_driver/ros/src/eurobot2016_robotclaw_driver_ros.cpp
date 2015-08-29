// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <eurobot2016_robotclaw_driver/eurobot2016_robotclaw_driverConfig.h>

// ROS message includes
#include <geometry_msgs/Twist.h>

// other includes
#include <eurobot2016_robotclaw_driver_common.cpp>


class eurobot2016_robotclaw_driver_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<eurobot2016_robotclaw_driver::eurobot2016_robotclaw_driverConfig> server;
    dynamic_reconfigure::Server<eurobot2016_robotclaw_driver::eurobot2016_robotclaw_driverConfig>::CallbackType f;

    ros::Subscriber cmd_vel_;

    eurobot2016_robotclaw_driver_data component_data_;
    eurobot2016_robotclaw_driver_config component_config_;
    eurobot2016_robotclaw_driver_impl component_implementation_;

    eurobot2016_robotclaw_driver_ros() : np_("~")
    {
        f = boost::bind(&eurobot2016_robotclaw_driver_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        cmd_vel_ = n_.subscribe("cmd_vel", 1, &eurobot2016_robotclaw_driver_impl::topicCallback_cmd_vel, &component_implementation_);

    }
    void topicCallback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
    {
        component_data_.in_cmd_vel = *msg;
    }

    void configure_callback(eurobot2016_robotclaw_driver::eurobot2016_robotclaw_driverConfig &config, uint32_t level)
    {
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

    ros::init(argc, argv, "eurobot2016_robotclaw_driver");

    eurobot2016_robotclaw_driver_ros node;
    node.configure();

 // if cycle time == 0 do a spin() here without calling node.update()
    ros::spin();

    return 0;
}
