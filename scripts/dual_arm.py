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


if __name__ == '__main__':

    sys.argv.append('joint_states:=/panda_dual/joint_states')
    rospy.init_node('ggg')
    
    mdp = MoveGroupPlanner()
    mdp.initial_pose()
    mdp.gripper_open("right")
    mdp.gripper_open("left")
    
    quat = (0, 0, 0.258819, 0.965926)
    euler = tf.transformations.euler_from_quaternion(quat)
    print(math.degrees(euler[2]))

    #STATE 1
    # joint_goal = [-1.40951, -1.76259, 1.90521, -2.75986, 1.96589, 3.00976, 0.769984, 0.136339, 0.0771, 0.132456, -1.61833, 0.163069, 1.64452, 2.04722]
    # joint_goal = [-2.56029, -0.96337, 1.25034, -1.24392, 0.933012, 1.34339, 1.33141, -0.260674, 0.494329, 0.109552, -1.73058, 1.25946, 1.46918, 1.61169]
    joint_goal = [-1.6671076987319884, -1.4984785565179355, 1.5404192524883924, -2.388776541995507, -2.897203095390305, 3.280959665933266, -1.0941728565641882, -0.10245023002256344, 0.26594811616937525, 0.41290315245014864, -1.390080227072562, 0.06799447874492352, 1.5907921066040371, 2.0916256998720577]
    mdp.plan_joint_target(joint_goal)
    
    for key, value in mdp.stefan.list.items():
            mdp.scene.add_mesh(key, value, mdp.stefan.stefan_dir + key + ".stl")
        
    rospy.sleep(1)
    touch_links = mdp.robot.get_link_names(group = 'panda_hands')
    mdp.scene.attach_mesh(mdp.group_left.get_end_effector_link(), "assembly", touch_links= touch_links)
    
    # mdp.gripper_close("left")
    # mdp.gripper_close("right")
    rospy.sleep(1)
    

    # mdp.plan_right_joint()
    # rospy.sleep(3)
    # mdp.gripper_open("right")
    # mdp.gripper_open("left")

