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

import yaml
import tf2_ros

from math import radians
from whole_part import SceneObject
from MoveGroupPlanner import MoveGroupPlanner
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


if __name__ == '__main__':

    sys.argv.append('joint_states:=/panda_dual/joint_states')
    rospy.init_node('ggg')

    mdp = MoveGroupPlanner()
    # mdp.gripper_open()
    
    goal_1st = [0.8538476617343256, 0.7765987891970887, -1.38718092553011, -1.9162352330353676, 2.693557656819878, 2.209230516957901, -2.8518449420397336]
    mdp.plan_joint_target(goal_1st, '1st')
    
    touch_links = mdp.robot.get_link_names(group='hand_chair_up')
    mdp.scene.attach_mesh(mdp.group_1st.get_end_effector_link(),
                          "assembly", touch_links=touch_links)
    mdp.scene.remove_attached_object(mdp.group_3rd.get_end_effector_link())

    rospy.sleep(1)
    mdp.initial_pose("3rd")
    goal_3rd = [2.4288973744080127, 0.2356190832102002, -2.6487764272706724, -2.409884568379378, 2.7754012268293335, 2.451555244441547, 2.786489214331766]

    mdp.plan_joint_target(goal_3rd, '3rd')
    rospy.sleep(1)
    mdp.scene.attach_mesh(mdp.group_3rd.get_end_effector_link(),
                          "assembly", touch_links=touch_links)


    mdp.initial_pose("2nd")
    # robot_trajectory = RobotTrajectory()
    # joint_trajectory = JointTrajectory()
    # joint_trajectory.joint_names = mdp.group_chairup.get_active_joints()
    # with open("/home/jiyeong/catkin_ws/projection_path.txt", 'r') as file:
    #     for line in file.readlines():
    #         data = line.split(" ")
    #         traj_point = JointTrajectoryPoint()
    #         for i in range(0, 14):
    #             traj_point.positions.append(data[i])
    #         joint_trajectory.points.append(traj_point)

    # n_traj = len(joint_trajectory.points)
    # for i in range(0, n_traj):
    #     joint_trajectory.points[i].time_from_start = rospy.Duration(
    #         2.0 / n_traj * i)

    # robot_trajectory.joint_trajectory = joint_trajectory
    
    # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    # display_trajectory.trajectory_start = mdp.robot.get_current_state()
    # display_trajectory.trajectory.append(robot_trajectory)
    # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
    #                                                moveit_msgs.msg.DisplayTrajectory,
    #                                                queue_size=20)

    # display_trajectory_publisher.publish(display_trajectory)
