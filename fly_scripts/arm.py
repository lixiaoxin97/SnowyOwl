import rospy 
import std_msgs.msg as std_msgs


class Arm:
    def __init__(self):
        rospy.init_node("fly_arm")
        self.rate = rospy.Rate(30)

        self._arm_bridge_pub = rospy.Publisher("/snowyowl/sbus_bridge/arm", std_msgs.Bool, queue_size=1)


    def arming(self):
        arm_message = std_msgs.Bool(True)
        for i in range(10):
            self._arm_bridge_pub.publish(arm_message)
            self.rate.sleep()
        
        rospy.loginfo("Armed!!!")

if __name__ == "__main__":
    arm_ = Arm()
    arm_.arming()
    