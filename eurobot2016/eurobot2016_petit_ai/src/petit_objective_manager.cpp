#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GetPlan.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "common_smart_nav/GetRobotPose.h"
#include "common_smart_nav/GetDistance.h"

#include "common_smart_ai/GetObjective.h"
#include "common_smart_ai/UpdatePriority.h"

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>                                         // for in-/output
#include <string.h>                                        // strcat
#include <fcntl.h>                                         // for 'O_RDONLY' deklaration
#include <termios.h>                                       // for serial

//Include system headers
#include <cstring>
#include <iostream>
#include <cstdio>

#include <sstream>
#include <math.h>

#include <vector>
#include <list>
#include <map>

#define DEFAULT_X	0.0
#define DEFAULT_Y	0.4
#define DEFAULT_A	0.0

using namespace std;


class ObjectiveManager {
    public:
        ObjectiveManager();
        void rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose);
        void loop(void);

        ros::Publisher goal_debug;
        ros::Publisher path_debug;

	ros::ServiceClient get_distance;

        ros::ServiceServer goal_service;

        ros::Subscriber delet_sub;
        ros::ServiceServer update_prio_service;

        ros::Subscriber color_sub;


    private:
        bool getObjective(common_smart_ai::GetObjective::Request  &req, common_smart_ai::GetObjective::Response &res );
        bool updateObjective(common_smart_ai::UpdatePriority::Request  &req, common_smart_ai::UpdatePriority::Response &res );
        void deletCallback(const geometry_msgs::PoseStamped::ConstPtr & pose);
        void fill_trees(void);
        void colorCallback(const std_msgs::Int32::ConstPtr & my_int);

        ros::NodeHandle nh;

        nav_msgs::Path my_path;

        pair<geometry_msgs::PoseStamped, uint32_t> best_objective; // First = goal position / Second = goal type

        list< pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t> > objectives;

        int color;
	std::string map_name;

};

ObjectiveManager::ObjectiveManager()
{
    ros::NodeHandle nhp("~");

    nhp.param<std::string>("map_name", map_name, "/map");

    color = 1;

/* TO MODIFY */
    goal_debug = nh.advertise < geometry_msgs::PoseStamped > ("/ROBOT/best_objective", 10);
    path_debug = nh.advertise < nav_msgs::Path > ("/ROBOT/debug_path", 10);

    get_distance = nh.serviceClient<common_smart_nav::GetDistance>("/ROBOT/get_distance");

    goal_service = nh.advertiseService("/ROBOT/get_objective", &ObjectiveManager::getObjective, this);

    delet_sub = nh.subscribe < geometry_msgs::PoseStamped > ("/ROBOT/delet_objective", 10, &ObjectiveManager::deletCallback, this);

    update_prio_service = nh.advertiseService("/ROBOT/update_priority", &ObjectiveManager::updateObjective, this);

    color_sub = nh.subscribe < std_msgs::Int32 > ("/GENERAL/color", 20, &ObjectiveManager::colorCallback, this);
/*****END*****/


    my_path.poses = std::vector < geometry_msgs::PoseStamped > ();

    if (my_path.poses.std::vector < geometry_msgs::PoseStamped >::size() >
            (my_path.poses.std::vector < geometry_msgs::PoseStamped >::max_size() - 2)) {
        my_path.poses.std::vector < geometry_msgs::PoseStamped >::pop_back();
    }

    best_objective.first.header.frame_id = map_name;
    best_objective.first.pose.position.x = DEFAULT_X;
    best_objective.first.pose.position.y = DEFAULT_Y;
    best_objective.first.pose.position.z = 0.0;
    best_objective.first.pose.orientation = tf::createQuaternionMsgFromYaw(DEFAULT_A);

}

void ObjectiveManager::fill_trees(void)
{   

    pair<geometry_msgs::PoseStamped, uint32_t> tmp_obj;
    tmp_obj.first.header.frame_id = map_name;
    tmp_obj.first.pose.position.z = 0;
    tmp_obj.first.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    tmp_obj.second = 1; // type of objective (1-X)

    // Get the 4 cubes 
    tmp_obj.first.pose.position.x = color*(1.500 - 0.650 + 0.100); //
    tmp_obj.first.pose.position.y = 2.000 - 0.900; 
    if(color<0)
    	tmp_obj.first.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    else
	tmp_obj.first.pose.orientation = tf::createQuaternionMsgFromYaw(3.1415); 
    tmp_obj.second = 1;
    // Priority : 
    objectives.push_back( pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t>(tmp_obj, 500) );

    // Release the cubes
    tmp_obj.first.pose.position.x = color*(1.500 - 1.300); //
    tmp_obj.first.pose.position.y = 1.000;
    if(color<0)
        tmp_obj.first.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    else
        tmp_obj.first.pose.orientation = tf::createQuaternionMsgFromYaw(3.1415);
    tmp_obj.second = 2;
    // Priority : 
    objectives.push_back( pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t>(tmp_obj, 300) );

    // Get shells next to the starting point
    tmp_obj.first.pose.position.x = color*(1.500 - 0.200); //
    tmp_obj.first.pose.position.y = 2.000 - 1.100;
    tmp_obj.first.pose.orientation = tf::createQuaternionMsgFromYaw(-1.57);
    tmp_obj.second = 6;
    // Priority : 
    objectives.push_back( pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t>(tmp_obj, 40) );

    // Push flag 1
    tmp_obj.first.pose.position.x = color*(1.500 - 0.300); //
    tmp_obj.first.pose.position.y = 2.000 - 0.200;
    tmp_obj.first.pose.orientation = tf::createQuaternionMsgFromYaw(-1.57);
    tmp_obj.second = 3;
    // Priority : 
    objectives.push_back( pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t>(tmp_obj, 50) );

    // Push flag 2
    tmp_obj.first.pose.position.x = color*(1.500 - 0.600); //
    tmp_obj.first.pose.position.y = 2.000 - 0.200;
    tmp_obj.first.pose.orientation = tf::createQuaternionMsgFromYaw(-1.57);
    tmp_obj.second = 3;
    // Priority : 
    objectives.push_back( pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t>(tmp_obj, 50) );

    // Take one shell
    tmp_obj.first.pose.position.x = color*(1.500 - 0.700); // 
    tmp_obj.first.pose.position.y = 2.000 - 1.250;
    tmp_obj.first.pose.orientation = tf::createQuaternionMsgFromYaw(-1.57); // VOIR POUR L'ANGLE
    tmp_obj.second = 5;
    // Priority : 
    objectives.push_back( pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t>(tmp_obj, 20) );

    // Get shells 
    tmp_obj.first.pose.position.x = color*(1.500 - 0.700); //
    tmp_obj.first.pose.position.y = 2.000 - 1.100;
    tmp_obj.first.pose.orientation = tf::createQuaternionMsgFromYaw(-1.57);
    tmp_obj.second = 6;
    // Priority : 
    objectives.push_back( pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t>(tmp_obj, 30) );

    // Take one shell
    tmp_obj.first.pose.position.x = color*(1.500 - 0.900); // 
    tmp_obj.first.pose.position.y = 2.000 - 1.450;
    tmp_obj.first.pose.orientation = tf::createQuaternionMsgFromYaw(-1.57); // VOIR POUR L'ANGLE
    tmp_obj.second = 5;
    // Priority : 
    objectives.push_back( pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t>(tmp_obj, 10) );

    // Take one shell
    tmp_obj.first.pose.position.x = color*(1.500 - 1.200); // 
    tmp_obj.first.pose.position.y = 2.000 - 1.650;
    tmp_obj.first.pose.orientation = tf::createQuaternionMsgFromYaw(-1.57); // VOIR POUR L'ANGLE
    tmp_obj.second = 5;
    // Priority : 
    objectives.push_back( pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t>(tmp_obj, 10) );

    // Release the shells
    tmp_obj.first.pose.position.x = color*(1.500 - 0.300); //
    tmp_obj.first.pose.position.y = 2.000 - 0.800;
    if(color>0)
        tmp_obj.first.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    else
        tmp_obj.first.pose.orientation = tf::createQuaternionMsgFromYaw(3.1415);
    tmp_obj.second = 2;
    // Priority : 
    objectives.push_back( pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t>(tmp_obj, 0) );


    tmp_obj.first.pose.position.x = color*(DEFAULT_X); // end
    tmp_obj.first.pose.position.y = DEFAULT_Y;
    tmp_obj.first.pose.orientation = tf::createQuaternionMsgFromYaw(DEFAULT_A);
    tmp_obj.second = 1;
    // Priority : 
    objectives.push_back( pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t>(tmp_obj, 1) );


}

void ObjectiveManager::colorCallback(const std_msgs::Int32::ConstPtr & my_int)
{
        if(my_int->data == 1) {
                color = my_int->data;
        }
        else { 
                color = -1;
        }

        fill_trees();
}



bool ObjectiveManager::getObjective(common_smart_ai::GetObjective::Request  &req,
        common_smart_ai::GetObjective::Response &res )
{
    std_msgs::Int32 tmp;
    tmp.data = best_objective.second;
    res.goal = best_objective.first;
    res.type = tmp;
    return true;
}

bool ObjectiveManager::updateObjective(common_smart_ai::UpdatePriority::Request  &req, common_smart_ai::UpdatePriority::Response &res )
{
    list< pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t> > tmp_list;
    list< pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t> > tmp_list2;

    tmp_list = objectives;

    while (!tmp_list.empty()) {

        if( (tmp_list.back().first.first.pose.position.x == req.goal.pose.position.x) && (tmp_list.back().first.first.pose.position.y == req.goal.pose.position.y) ) {

            tmp_list2.push_back( pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t>(tmp_list.back().first, tmp_list.back().second + req.prio.data) );
        }
        else {
            tmp_list2.push_back(tmp_list.back());
        }
        tmp_list.pop_back();

    }

    objectives = tmp_list2;
}

void ObjectiveManager::deletCallback(const geometry_msgs::PoseStamped::ConstPtr & pose) 
{
    list< pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t> > tmp_list;
    list< pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t> > tmp_list2;

    tmp_list = objectives;

    while (!tmp_list.empty()) {

        if( (tmp_list.back().first.first.pose.position.x == pose->pose.position.x) && (tmp_list.back().first.first.pose.position.y == pose->pose.position.y) ) {
		
		// Objectives that cannot be deleted (only put the priority to 0)
	        if( (tmp_list.back().first.first.pose.position.x == (color*(1.500 - 0.250)) ) && (tmp_list.back().first.first.pose.position.y == 0.800) ) {
			tmp_list2.push_back( pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t>(tmp_list.back().first, 0) );
		}

        }
        else {
            tmp_list2.push_back(tmp_list.back());
        }
        tmp_list.pop_back();

    }

    objectives = tmp_list2;

    ObjectiveManager::loop();
}


void ObjectiveManager::loop(void) 
{
    double best_prio = 0;
    double current_prio = 0;

    nav_msgs::GetPlan tmp_plan;

    common_smart_nav::GetRobotPose tmp_pose;
    common_smart_nav::GetDistance tmp_distance;

    //loop
    for( list< pair<pair<geometry_msgs::PoseStamped, uint32_t>, uint32_t> >::iterator iter = objectives.begin(); iter != objectives.end(); iter++ ) {
        current_prio = 0;

        tmp_distance.request.goal = iter->first.first;
        if (get_distance.call(tmp_distance))
        {
            current_prio = (3.0) / ((tmp_distance.response.distance.data));
              if(current_prio < 10000000.0) {
                  // add it to the base priority
                  if((double)iter->second > 0.0)
                          current_prio = current_prio + (double)iter->second;
                  else
                          current_prio = 0.0;
                  //ROS_ERROR("Prio tot : %f", current_prio);
                  if(current_prio > best_prio) {
                      best_prio = current_prio;
                      best_objective = iter->first;
                  }
              }

	}
	else 
	{
		ROS_ERROR("Failed to call service GetDistance");
	}

        //usleep(200000); // For debug purpose
    }
    // end loop

    // if no objective available, make random movements
    if(best_prio > 10000000.0) {
	best_objective.first.pose.position.x = DEFAULT_X;
    	best_objective.first.pose.position.y = DEFAULT_Y;
    	best_objective.first.pose.orientation = tf::createQuaternionMsgFromYaw(DEFAULT_A);
    }

    // For debug purpose
    goal_debug.publish(best_objective.first);

    //ROS_INFO("Trajectory manager : Goal sent.");

}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{


    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line. For programmatic
     * remappings you can use a different version of init() which takes remappings
     * directly, but for most command-line programs, passing argc and argv is the easiest
     * way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
/* TO MODIFY */
    ros::init(argc, argv, "ROBOT_objective_manager");
/* TO MODIFY */
    ObjectiveManager objectivemanager;
    tf::TransformListener listener(ros::Duration(10));

    // Refresh rate
    ros::Rate loop_rate(2);
    float rotation = 0.0;
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
        objectivemanager.loop();
    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}



