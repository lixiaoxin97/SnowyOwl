import rospy 
import std_msgs.msg as std_msgs


class Start:
    def __init__(self):
        rospy.init_node("fly_start")
        self.rate = rospy.Rate(30)

        self._start_pub = rospy.Publisher("snowyowl/autopilot/start", std_msgs.Empty, queue_size=1)


    def starting(self):
        start_message = std_msgs.Empty()
        for i in range(10):
            self._start_pub.publish(start_message)
            self.rate.sleep()
        
        rospy.loginfo("Started!!!")

if __name__ == "__main__":
    start_ = Start()
    start_.starting()
