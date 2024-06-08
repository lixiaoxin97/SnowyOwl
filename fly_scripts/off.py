import rospy 
import std_msgs.msg as std_msgs


class Off:
    def __init__(self):
        rospy.init_node("fly_off")
        self.rate = rospy.Rate(30)

        self._off_pub = rospy.Publisher("snowyowl/autopilot/off", std_msgs.Empty, queue_size=1)


    def offing(self):
        off_message = std_msgs.Empty()
        for i in range(10):
            self._off_pub.publish(off_message)
            self.rate.sleep()
        
        rospy.loginfo("Offed!!!")

if __name__ == "__main__":
    off_ = Off()
    off_.offing()
