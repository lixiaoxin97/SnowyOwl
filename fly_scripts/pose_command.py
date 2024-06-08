import rospy 
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import math


class Pose_command:
    def __init__(self):
        rospy.init_node("fly_pose_command")
        self.rate = rospy.Rate(30)

        self._go_to_pose_pub = rospy.Publisher("snowyowl/autopilot/pose_command", geometry_msgs.PoseStamped, queue_size=1)

    def pose_commanding(self):
        go_to_pose_msg = geometry_msgs.PoseStamped()
        go_to_pose_msg.pose.position.x = float(1.0)
        go_to_pose_msg.pose.position.y = float(1.0)
        go_to_pose_msg.pose.position.z = float(1.0)

        heading = float(0.0) / 180.0 * math.pi

        go_to_pose_msg.pose.orientation.w = math.cos(heading / 2.0)
        go_to_pose_msg.pose.orientation.z = math.sin(heading / 2.0)
        for i in range(10):
            self._go_to_pose_pub.publish(go_to_pose_msg)
            self.rate.sleep()
        
        rospy.loginfo("Pose_commanded!!!")

if __name__ == "__main__":
    pose_command_ = Pose_command()
    pose_command_.pose_commanding()