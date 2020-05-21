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
    mdp.gripper_open()
    closed_chain = False
    if closed_chain:
        joint_goal = [0.8538476617343256, 0.7765987891970887, -1.38718092553011, -1.9162352330353676, 2.693557656819878, 2.209230516957901, -2.8518449420397336,
                      2.4288973744080127, 0.2356190832102002, -2.6487764272706724, -2.409884568379378, 2.7754012268293335, 2.451555244441547, 2.786489214331766]
        touch_links = mdp.robot.get_link_names(group='hand_closed_chain')
        mdp.plan_joint_target(joint_goal)

    else:
        joint_goal = [1.7635811732933235, -1.4411345207422865, -1.964651184080014, -1.7905553615439762, 0.20378384311742412, 1.7390337027885823, -2.800300667744541, -
                      2.507227794231461, -0.23624109784362163, 2.5633123073239905, -2.268388140289912, 0.24936065684482742, 2.4538909693928335, -0.9104041928398361]
        touch_links = mdp.robot.get_link_names(group='hand_chair_up')
        mdp.plan_joint_target(joint_goal, 'panda_chair_up')
    rospy.sleep(2)
    for i in joint_goal :
        print(i * 180 / pi)
    for key, value in mdp.stefan.list.items():
            mdp.scene.add_mesh(
                key, value, mdp.stefan.stefan_dir + key + ".stl")

    rospy.sleep(1)
    mdp.scene.attach_mesh(mdp.group_3rd.get_end_effector_link(),
                          "assembly", touch_links=touch_links)

    # mdp.gripper_close("left")
    # mdp.gripper_close("right")
    rospy.sleep(1)

    # mdp.plan_right_joint()
    # rospy.sleep(3)
    # mdp.gripper_open("right")
    # mdp.gripper_open("left")
