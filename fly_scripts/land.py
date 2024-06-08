import rospy 
import std_msgs.msg as std_msgs


class Land:
    def __init__(self):
        rospy.init_node("fly_land")
        self.rate = rospy.Rate(30)

        self._land_pub = rospy.Publisher("snowyowl/autopilot/land", std_msgs.Empty, queue_size=1)


    def landing(self):
        land_message = std_msgs.Empty()
        for i in range(10):
            self._land_pub.publish(land_message)
            self.rate.sleep()
        
        rospy.loginfo("Landed!!!")

if __name__ == "__main__":
    land_ = Land()
    land_.landing()
