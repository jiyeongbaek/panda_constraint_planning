#!/usr/bin/env python
from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander

import actionlib

import std_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg
import tf_conversions
import math
import franka_control.srv
from math import pi
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_matrix, rotation_matrix, translation_from_matrix, quaternion_matrix
import tf
from tf.listener import TransformerROS, Transformer
import numpy as np

from whole_part import SceneObject

import yaml
import tf2_ros
class MoveGroupPlanner():
    def __init__(self):
        ### MoveIt! 
        moveit_commander.roscpp_initialize(sys.argv)
    
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group = moveit_commander.MoveGroupCommander("panda_arms")
        self.group_left = moveit_commander.MoveGroupCommander("panda_left")
        self.group_right = moveit_commander.MoveGroupCommander("panda_right")
        
        self.hand_left = moveit_commander.MoveGroupCommander("hand_left")
        self.hand_right = moveit_commander.MoveGroupCommander("hand_right")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.group.get_planning_frame()
        print ("============ Reference frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.group.get_end_effector_link()
        print ("============ End effector: %s" % self.eef_link)

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print ("============ Robot Groups:", self.robot.get_group_names())

        rospy.sleep(1)
        self.stefan = SceneObject()

        self.scene.remove_world_object()
        self.scene.remove_attached_object(self.group.get_end_effector_link())

        # for key, value in self.stefan.list.items():
        #     self.scene.add_mesh(key, value, self.stefan_dir + key + ".stl")
        # rospy.sleep(1)
        
        ### Franka Collision
        self.set_collision_behavior = rospy.ServiceProxy(
            'franka_control/set_force_torque_collision_behavior',
            franka_control.srv.SetForceTorqueCollisionBehavior)
        # self.set_collision_behavior.wait_for_service()

        self.active_controllers = []


    # geometry_msgs.msg.Pose() or self.group.get_current_joint_values()
    def plan(self, goal, arm_name):
        if (arm_name == 'panda_arms'):
            self.group.set_max_velocity_scaling_factor = 0.6
            self.group.set_max_acceleration_scaling_factor = 0.4
            self.group.set_start_state_to_current_state()
            trajectory = self.group.plan(goal)
        return trajectory
    
    # def plan_cartesian_target(self):
    #     pose_goal = geometry_msgs.msg.Pose()
    #     pose_goal.orientation = geometry_msgs.msg.Quaternion(
    #         *tf_conversions.transformations.quaternion_from_euler(math.radians(90), math.radians(90), math.radians(0)))
    #     pose_goal.position.x =  0.5429
    #     pose_goal.position.y = 0.05
    #     pose_goal.position.z = 0.6 + 0.30

    #     trajectory = self.group_left.plan(pose_goal)
    #     return trajectory

    def initial_pose(self):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[6] = pi/4
        joint_goal[13] = pi/4
        self.group.plan(joint_goal)
        self.group.go()
        
    def plan_joint_target(self, joint_goal, arm_name='panda_arms'):
        if (len(joint_goal) == 14):
            self.group.plan(joint_goal)
            self.group.go()

        elif (arm_name=='left'):
            self.group_left.plan(joint_goal)
            self.group_left.go()
        elif ( arm_name == 'right'):
            self.group_right.plan(joint_goal)
            self.group_right.go()




    def gripper_open(self, arm):
        joint_goal = self.hand_left.get_current_joint_values()
        joint_goal[0] = 0.04
        joint_goal[1] = 0.04
        if (arm == "left"):
            self.hand_left.plan(joint_goal)
            print("============ OPEN LEFT GRIPPER")
            self.hand_left.go()
        else :
            self.hand_right.plan(joint_goal)
            print("============ OPEN RIGHT GRIPPER")
            self.hand_right.go()
        

    def gripper_close(self, arm):
        joint_goal = self.hand_left.get_current_joint_values()
        joint_goal[0] = 0.03
        joint_goal[1] = 0.03
        if (arm == "left"):
            plan = self.hand_left.plan(joint_goal)
            if plan.joint_trajectory.points:
                self.hand_left.go()
                print("CLOSE LEFT GRIPPER")
        else :
            plan = self.hand_right.plan(joint_goal)
            if plan.joint_trajectory.points:
                self.hand_right.go()
                print("CLOSE RIGHT GRIPPER")

    def display_trajectory(self, plan):
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)
