# -*- coding: UTF-8 -*-
""" ***************************************************************************************
This is a node to bridge Vicon and Autopilot.
* subscribe pose and twist data from Vicon and publish to state_estimate 

@ environment: ros
@ auther: Guanzheng Wang
@ creation date: Nov.14, 2022
*************************************************************************************** """ 

import rospy 
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry


class ViconTransfer:
    def __init__(self):
        rospy.init_node("vicon_transfer_")
        self.rate = rospy.Rate(50)
        self.vision_pose = PoseStamped()
        self.vision_twist = TwistStamped()
        self.odom = Odometry()

        self.vicon_pose_sub = rospy.Subscriber("/vrpn_client_node/snowyowl3/pose", PoseStamped, self.vicon_pose_callback)
        self.vicon_twist_sub = rospy.Subscriber("/vrpn_client_node/snowyowl3/twist", TwistStamped, self.vicon_twist_callback)

        self.odom_pub = rospy.Publisher("/snowyowl3/autopilot/state_estimate", Odometry, queue_size=5)

        rospy.loginfo("Vicon Transfer node initialized!")

    def vicon_pose_callback(self, msg):
        self.vision_pose = msg

    def vicon_twist_callback(self, msg):
        self.vision_twist = msg 

    def clue(self):
        
        self.odom.header.frame_id = "world"
        self.odom.child_frame_id = "snowyowl3/base_link"

        self.odom.pose.pose.position.x = self.vision_pose.pose.position.x
        self.odom.pose.pose.position.y = self.vision_pose.pose.position.y
        self.odom.pose.pose.position.z = self.vision_pose.pose.position.z
        self.odom.pose.pose.orientation.x = self.vision_pose.pose.orientation.x
        self.odom.pose.pose.orientation.y = self.vision_pose.pose.orientation.y
        self.odom.pose.pose.orientation.z = self.vision_pose.pose.orientation.z
        self.odom.pose.pose.orientation.w = self.vision_pose.pose.orientation.w
        self.odom.twist.twist.linear.x= self.vision_twist.twist.linear.x
        self.odom.twist.twist.linear.y= self.vision_twist.twist.linear.y
        self.odom.twist.twist.linear.z= self.vision_twist.twist.linear.z
        self.odom.twist.twist.angular.x= self.vision_twist.twist.angular.x
        self.odom.twist.twist.angular.y= self.vision_twist.twist.angular.y
        self.odom.twist.twist.angular.z= self.vision_twist.twist.angular.z
        self.odom.header.stamp = rospy.Time.now()

    def transfering(self):
        rospy.loginfo("Ready to transfer data from Vicon System!")
        while not rospy.is_shutdown():
            self.clue()
            self.odom_pub.publish(self.odom)
            self.rate.sleep()

if __name__ == "__main__":
    vicon_transfer = ViconTransfer()
    vicon_transfer.transfering()

        
