import geometry_msgs.msg
from tf.listener import TransformerROS
import tf
import rospy
import moveit_msgs.msg
from math import radians
class SceneObject():
    def __init__(self):
        self.stefan_dir = "/home/jiyeong/catkin_ws/src/1_assembly/grasping_point/STEFAN/stl/"
        # self.stefan_dir = "package://STEFAN/stl/"
        self.assembly = "assembly"
        self.assembly_pose = geometry_msgs.msg.PoseStamped()
        self.assembly_pose.header.frame_id="base"
        # self.assembly_pose.pose.orientation.w = 1.0

        # #STATE 1
        self.assembly_pose.pose.position.x = 0.95
        self.assembly_pose.pose.position.y = 0.0 
        self.assembly_pose.pose.position.z = 0.72 #0.601
        self.assembly_pose.pose.orientation.x = 0
        self.assembly_pose.pose.orientation.y = 0
        self.assembly_pose.pose.orientation.z = 0.258819
        self.assembly_pose.pose.orientation.w = 0.965926

        #STATE 2
        # self.assembly_pose.pose.position.x = 0.965529
        # self.assembly_pose.pose.position.y = 0.0510316
        # self.assembly_pose.pose.position.z = 0.654535
        # self.assembly_pose.pose.orientation.x = 0.366184
        # self.assembly_pose.pose.orientation.y = 0.111416
        # self.assembly_pose.pose.orientation.z = 0.112839
        # self.assembly_pose.pose.orientation.w = 0.916932

        #STATE 3
        # self.assembly_pose.pose.position.x = 0.966875
        # self.assembly_pose.pose.position.y = 0.0554544
        # self.assembly_pose.pose.position.z =0.648861
        # self.assembly_pose.pose.orientation.x = 0.395828
        # self.assembly_pose.pose.orientation.y = 0.120436
        # self.assembly_pose.pose.orientation.z = 0.0988886
        # self.assembly_pose.pose.orientation.w = 0.905006


        # self.assembly_pose.pose.position.x = 1
        # self.assembly_pose.pose.position.y = 0.18
        # self.assembly_pose.pose.position.z =0.72
        # self.assembly_pose.pose.orientation.x = 0.675359
        # self.assembly_pose.pose.orientation.y = 0.0192997
        # self.assembly_pose.pose.orientation.z = 0.0176849
        # self.assembly_pose.pose.orientation.w = 0.737025


        # TEST MODE
        self.list = {self.assembly : self.assembly_pose}
     