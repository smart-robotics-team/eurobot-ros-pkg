// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <beacon_pt_filter/beacon_pt_filterConfig.h>

// ROS message includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

// other includes
#include <beacon_pt_filter_common.cpp>


class beacon_pt_filter_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<beacon_pt_filter::beacon_pt_filterConfig> server;
    dynamic_reconfigure::Server<beacon_pt_filter::beacon_pt_filterConfig>::CallbackType f;

    ros::Publisher output_;
    ros::Subscriber input_;

    beacon_pt_filter_data component_data_;
    beacon_pt_filter_config component_config_;
    beacon_pt_filter_impl component_implementation_;

    beacon_pt_filter_ros() : np_("~")
    {
        f = boost::bind(&beacon_pt_filter_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        output_ = n_.advertise<sensor_msgs::PointCloud2>("output", 1);
        input_ = n_.subscribe("input", 1, &beacon_pt_filter_impl::topicCallback_input, &component_implementation_);

        np_.param("x_min", component_config_.x_min, (double)-1.5);
        np_.param("x_max", component_config_.x_max, (double)1.5);
        np_.param("y_min", component_config_.y_min, (double)0);
        np_.param("y_max", component_config_.y_max, (double)2.0);
        np_.param("world_frame_id", component_config_.world_frame_id, (std::string)"world");
    }
    void topicCallback_input(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        component_data_.in_input = *msg;
    }

    void configure_callback(beacon_pt_filter::beacon_pt_filterConfig &config, uint32_t level)
    {
        component_config_.x_min = config.x_min;
        component_config_.x_max = config.x_max;
        component_config_.y_min = config.y_min;
        component_config_.y_max = config.y_max;
        component_config_.world_frame_id = config.world_frame_id;
        configure();
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_output_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_output_active)
            output_.publish(component_data_.out_output);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "beacon_pt_filter");

    beacon_pt_filter_ros node;
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
