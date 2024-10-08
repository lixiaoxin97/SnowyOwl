import rospy 
import std_msgs.msg as std_msgs


class Force_hover:
    def __init__(self):
        rospy.init_node("fly_force_hover")
        self.rate = rospy.Rate(30)

        self._force_hover_pub = rospy.Publisher("snowyowl3/autopilot/force_hover", std_msgs.Empty, queue_size=1)


    def force_hovering(self):
        force_hover_message = std_msgs.Empty()
        for i in range(10):
            self._force_hover_pub.publish(force_hover_message)
            self.rate.sleep()
        
        rospy.loginfo("Force_hovered!!!")

if __name__ == "__main__":
    force_hover_ = Force_hover()
    force_hover_.force_hovering()
