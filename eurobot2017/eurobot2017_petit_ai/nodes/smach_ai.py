#!/usr/bin/env python

""" smach_ai.py - Version 1.0

    Control a robot

    Copyright (c) 2016 Joffrey KRIEGEL.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""

import rospy
import actionlib
import smach
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from actionlib import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from std_msgs.msg import UInt16, Int32, Empty
from random import randrange
from tf.transformations import quaternion_from_euler
from math import  pi
from collections import OrderedDict
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from common_smart_ai.srv import GetObjective, UpdatePriority, UpdatePriorityRequest

class Stop(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])

	self.cmd_vel_pub = rospy.Publisher('/PETIT/cmd_vel', Twist)
        pass

    def execute(self, userdata):
	self.cmd_vel_pub.publish(Twist())
        rospy.loginfo("Shutting down the state machine")
        return 'succeeded'

class Pause(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        pass

    def execute(self, userdata):
        rospy.loginfo("Thinking...")
        rospy.sleep(0.5)   
        return 'succeeded'

        
class Nav2Waypoint(State):
    def __init__(self, waypoint):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("PETIT_pathplanner", MoveBaseAction)
        self.waypoint = waypoint
        # Wait up to 60 seconds for the action server to become available
        # TODO  
        # TODO  
        # TODO  
        # TODO A REMETTRE !!!!! self.move_base.wait_for_server(rospy.Duration(60))    
        # TODO  
        # TODO  
        # TODO  
        rospy.loginfo("Connected to move_base action server")
        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'petit_map'

    def execute(self, userdata):
        self.goal.target_pose = self.waypoint

	rospy.loginfo("waypoint_in: " + str(self.waypoint))

        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(self.goal)
        
	i = 0
	finished_within_time = 0
	while i < 60 and not finished_within_time:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
        
            # Allow 1 minute to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(1)) 
	    #rospy.loginfo('finished ' + str(finished_within_time))
        
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return 'aborted'
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                #rospy.sleep(1)   
            return 'succeeded'

class MoveForward(State):
    def __init__(self, value):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])

	self.value = value
        self.distance_pub = rospy.Publisher('/PETIT/delta_ardu', Int32)
        self.pause_pub = rospy.Publisher('/PETIT/pause_pathwrapper', Empty)
        self.resume_pub = rospy.Publisher('/PETIT/resume_pathwrapper', Empty)
        pass

    def execute(self, userdata):
        rospy.loginfo("moving forward")
        self.pause_pub.publish(Empty())
        rospy.sleep(0.1)   
	tmp_distance = Int32()
	tmp_distance.data = value * 1000
        self.distance_pub.publish(tmp_distance)
        rospy.sleep(4)
        self.resume_pub.publish(Empty())
        return 'succeeded'


class CalibX(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])

        self.calibrate_pub = rospy.Publisher('/PETIT/calibrate', Int32)
        pass

    def execute(self, userdata):
        rospy.loginfo("X calibration")
        tmp_int = Int32()
        tmp_int.data = 0
        self.calibrate_pub.publish(tmp_int)
        rospy.sleep(20)
        return 'succeeded'

class CalibY(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])

        self.calibrate_pub = rospy.Publisher('/PETIT/calibrate', Int32)
        pass
    
    def execute(self, userdata):
        rospy.loginfo("Y calibration")
        tmp_int = Int32()
        tmp_int.data = 1
        self.calibrate_pub.publish(tmp_int)
        rospy.sleep(20)
        return 'succeeded'

class Forks(State):
    def __init__(self, value):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
	
	self.value = value
        self.forks_pub = rospy.Publisher('/PETIT/left_servo', Int32)
        pass

    def execute(self, userdata):
        rospy.loginfo("Move forks")
        tmp_int = Int32()
        tmp_int.data = self.value
        self.forks_pub.publish(tmp_int)
        rospy.sleep(0.5)
        return 'succeeded'






class SMACHAI():
    def __init__(self):
        rospy.init_node('petit_smach_ai', anonymous=False)
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        

        # Create a list to hold the target quaternions (orientations)
        quaternions = list()

        # First define the corner orientations as Euler angles
        euler_angles = (0, pi, pi/2, pi, pi/4)

        # Then convert the angles to quaternions
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)

        # Create a list to hold the waypoint poses
        self.waypoints = list()

	self.square_size = 1.0

        # Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        self.waypoints.append(Pose(Point(0.4, 0.65, 0.0), quaternions[0]))
        self.waypoints.append(Pose(Point(1.1, 0.50, 0.0), quaternions[1]))
        self.waypoints.append(Pose(Point(0.4, 0.35, 0.0), quaternions[2]))
        self.waypoints.append(Pose(Point(1.0, 0.65, 0.0), quaternions[3]))
        self.waypoints.append(Pose(Point(1.2, 0.30, 0.0), quaternions[4]))

	# Publisher to manually control the robot (e.g. to stop it)
    	self.cmd_vel_pub = rospy.Publisher('/PETIT/cmd_vel', Twist)

        self.stopping = False
        self.recharging = False

        self.robot_side = 1




	# State machine for Actions
        self.sm_actions = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with self.sm_actions:
	    # Calib X 
	    StateMachine.add('CALIBRATE_X', CalibX(),
                             transitions={'succeeded':'CALIBRATE_Y',
                                          'aborted':'aborted'})
	    # Calib Y
	    StateMachine.add('CALIBRATE_Y', CalibY(),
                             transitions={'succeeded':'GOTO_1_1',
                                          'aborted':'aborted'})
	    # NavPoint 1 
	    StateMachine.add('GOTO_1_1', Nav2Waypoint(self.waypoints[0]),
                             transitions={'succeeded':'FORWARD_1',
                                          'aborted':'aborted'})
	    # Avancer 
	    StateMachine.add('FORWARD_1', MoveForward(0.15),
                             transitions={'succeeded':'FORKS_10',
                                          'aborted':'aborted'})
	    # Monter fourches
	    StateMachine.add('FORKS_10', Forks(90),
                             transitions={'succeeded':'FORKS_11',
                                          'aborted':'aborted'})
	    StateMachine.add('FORKS_11', Forks(85),
                             transitions={'succeeded':'BACKWARD_1',
                                          'aborted':'aborted'})
	    # Reculer
	    StateMachine.add('BACKWARD_1', MoveForward(-0.15),
                             transitions={'succeeded':'GOTO_2_1',
                                          'aborted':'aborted'})
	    # NavPoint 2
	    StateMachine.add('GOTO_2_1', Nav2Waypoint(self.waypoints[1]),
                             transitions={'succeeded':'FORWARD_2',
                                          'aborted':'aborted'})
	    # Avancer
	    StateMachine.add('FORWARD_2', MoveForward(0.15),
                             transitions={'succeeded':'FORKS_20',
                                          'aborted':'aborted'})
	    # Baisser fourches
	    StateMachine.add('FORKS_20', Forks(90),
                             transitions={'succeeded':'FORKS_21',
                                          'aborted':'aborted'})
	    StateMachine.add('FORKS_21', Forks(95),
                             transitions={'succeeded':'BACKWARD_2',
                                          'aborted':'aborted'})
	    # Reculer
	    StateMachine.add('BACKWARD_2', MoveForward(-0.15),
                             transitions={'succeeded':'GOTO_3_1',
                                          'aborted':'aborted'})
	    # NavPoint 3
	    StateMachine.add('GOTO_3_1', Nav2Waypoint(self.waypoints[2]),
                             transitions={'succeeded':'GOTO_4_1',
                                          'aborted':'aborted'})
	    # NavPoint 4
	    StateMachine.add('GOTO_4_1', Nav2Waypoint(self.waypoints[3]),
                             transitions={'succeeded':'GOTO_2_2',
                                          'aborted':'aborted'})
	    # NavPoint 2
	    StateMachine.add('GOTO_2_2', Nav2Waypoint(self.waypoints[1]),
                             transitions={'succeeded':'FORWARD_3',
                                          'aborted':'aborted'})
	    # Avancer
	    StateMachine.add('FORWARD_3', MoveForward(0.15),
                             transitions={'succeeded':'FORKS_30',
                                          'aborted':'aborted'})
	    # Monter fourches
	    StateMachine.add('FORKS_30', Forks(90),
                             transitions={'succeeded':'FORKS_31',
                                          'aborted':'aborted'})
	    StateMachine.add('FORKS_31', Forks(85),
                             transitions={'succeeded':'BACKWARD_3',
                                          'aborted':'aborted'})
	    # Reculer
	    StateMachine.add('BACKWARD_3', MoveForward(-0.15),
                             transitions={'succeeded':'GOTO_1_2',
                                          'aborted':'aborted'})
	    # NavPoint 1
	    StateMachine.add('GOTO_1_2', Nav2Waypoint(self.waypoints[0]),
                             transitions={'succeeded':'FORWARD_4',
                                          'aborted':'aborted'})
	    # Avancer
	    StateMachine.add('FORWARD_4', MoveForward(0.15),
                             transitions={'succeeded':'FORKS_40',
                                          'aborted':'aborted'})
	    # Baisser fourches
	    StateMachine.add('FORKS_40', Forks(90),
                             transitions={'succeeded':'FORKS_41',
                                          'aborted':'aborted'})
	    StateMachine.add('FORKS_41', Forks(95),
                             transitions={'succeeded':'BACKWARD_4',
                                          'aborted':'aborted'})
	    # Reculer
	    StateMachine.add('BACKWARD_4', MoveForward(-0.15),
                             transitions={'succeeded':'FORKS_10',
                                          'aborted':'aborted'})
	    # NavPoint 5
	    StateMachine.add('GOTO_5_1', Nav2Waypoint(self.waypoints[4]),
                             transitions={'succeeded':'CALIBRATE_X',
                                          'aborted':'aborted'})




        # Create the top level state machine
        self.sm_top = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

        # Add nav_patrol, sm_recharge and a Stop() machine to sm_top
        with self.sm_top:
	    StateMachine.add('ACTIONS', self.sm_actions, transitions={'succeeded':'ACTIONS', 'aborted':'STOP'})
            #StateMachine.add('RECHARGE', self.sm_recharge, transitions={'succeeded':'PATROL'})
            StateMachine.add('STOP', Stop(), transitions={'succeeded':''})








        # Create and start the SMACH introspection server
        intro_server = IntrospectionServer('patrol', self.sm_top, '/SM_ROOT')
        intro_server.start()
        
        # Execute the state machine
        sm_outcome = self.sm_top.execute()
        
        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))
                
        intro_server.stop()


    def time_cb(self, userdata, msg):
        if msg.data < 2:
            self.stopping = True
            return False
        else:
            self.stopping = False
            return True

    def start_cb(self, userdata, msg):
	rospy.loginfo("Start !")
        return False

    def color_cb(self, userdata, msg):
        rospy.loginfo("Color " + str(msg.data))
	self.robot_side = msg.data

	self.sm_action1.userdata.robot_side = self.robot_side
	self.sm_action2.userdata.robot_side = self.robot_side
	self.sm_action3.userdata.robot_side = self.robot_side
	self.sm_action4.userdata.robot_side = self.robot_side
	self.sm_action5.userdata.robot_side = self.robot_side
	self.sm_action6.userdata.robot_side = self.robot_side
	self.sm_action7.userdata.robot_side = self.robot_side
	
	self.sm_top.userdata.robot_side = self.robot_side # TODO REMOVE

        return False

    def battery_cb(self, userdata, msg):
        if msg.data < 320:
            self.recharging = True
            return False
        else:
            self.recharging = False
            return True

    def objective_cb(self, userdata, response):
        #objective_response = GetObjective().Response
	userdata.waypoint_out = response.goal
	waypoint_type = response.type.data

	rospy.loginfo("goal: " + str(response.goal))

	if(waypoint_type == 1):
                return 'action1'
        if(waypoint_type == 2):
                return 'action2'
        if(waypoint_type == 3):
                return 'action3'
        if(waypoint_type == 4):
                return 'action4'
        if(waypoint_type == 5):
                return 'action5'
        if(waypoint_type == 6):
                return 'action6'
        if(waypoint_type == 7):
                return 'action7'
        return 'aborted'
	
    def requestPrioCube_cb(self, userdata, request):
        rospy.loginfo("Side : " + str(userdata.robot_side))
	update_request = UpdatePriorityRequest()
        update_request.goal.pose.position.x = userdata.robot_side*(1.500 - 1.300)
        update_request.goal.pose.position.y = 1.000
        update_request.prio.data = 100
        rospy.loginfo("Request Priority Drop cubes update")
        return update_request
	#request.goal.pose.position.x = userdata.robot_side*(1.500 - 1.300)
	#request.goal.pose.position.y = 1.000
	#request.prio = 100
	#return request


    def updatePrioCube_cb(self, userdata, response):
        rospy.loginfo("Priority Drop cubes updated")
        return 'aborted'


    def requestPrioShell_cb(self, userdata, request):
	rospy.loginfo("Side : " + str(userdata.robot_side))
        update_request = UpdatePriorityRequest()
        update_request.goal.pose.position.x = userdata.robot_side*(1.500 - 0.300)
        update_request.goal.pose.position.y = 0.800
        update_request.prio.data = 100
        rospy.loginfo("Request Priority Drop shell update")
        return update_request
	#request.goal.pose.position.x = userdata.robot_side*(1.500 - 0.300)
	#request.goal.pose.position.y = 0.800
	#request.prio = 100
	#return request

    def updatePrioShell_cb(self, userdata, response):
        rospy.loginfo("Priority Drop shell updated")
        return 'aborted'


    # Gets called when ANY child state terminates
    def concurrence_child_termination_cb(self, outcome_map):
        # If the current navigation task has succeeded, return True
        if outcome_map['SM_ACTIONS'] == 'succeeded':
            return True
        # If the MonitorState state returns False (invalid), store the current nav goal and recharge
        if outcome_map['MONITOR_TIME'] == 'invalid':
            rospy.loginfo("LOW TIME! NEED TO STOP...")
            return True
        if outcome_map['MONITOR_BATTERY'] == 'invalid':
            rospy.loginfo("LOW BATTERY! NEED TO STOP...")
            return True
        else:
            return False


    # Gets called when ALL child states are terminated
    def concurrence_outcome_cb(self, outcome_map):
        # If the battery is below threshold, return the 'recharge' outcome
        if outcome_map['MONITOR_TIME'] == 'invalid':
	    rospy.loginfo("TIME FINISHED !! GOING TO STOP ! ")
            return 'stop'
        if outcome_map['MONITOR_BATTERY'] == 'invalid':
            return 'stop'
        # Otherwise, if the last nav goal succeeded, return 'succeeded' or 'stop'
        elif outcome_map['SM_ACTIONS'] == 'succeeded':
            #self.patrol_count += 1
            #rospy.loginfo("FINISHED PATROL LOOP: " + str(self.patrol_count))
            # If we have not completed all our patrols, start again at the beginning
            #if self.n_patrols == -1 or self.patrol_count < self.n_patrols:
                #self.sm_nav.set_initial_state(['NAV_STATE_0'], UserData())
            return 'succeeded'
            # Otherwise, we are finished patrolling so return 'stop'
            #else:
                #self.sm_nav.set_initial_state(['NAV_STATE_4'], UserData())
                #return 'stop'
        # Recharge if all else fails
        else:
            return 'recharge'


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        
        self.sm_actions.request_preempt()
        
        self.cmd_vel_pub.publish(Twist())
        
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        SMACHAI()
    except rospy.ROSInterruptException:
        rospy.loginfo("Petit AI finished.")
