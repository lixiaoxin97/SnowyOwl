# -*- coding: UTF-8 -*-
""" ***************************************************************************************
This is a node to control the quadrotor through neural network.
* "state_estimate"
* "control_command" "bridge/arm"

@ environment: python3, tensorflow and ros
@ auther: Xiaoxin Li
@ creation date: Mar.29, 2024
*************************************************************************************** """ 

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
import std_msgs.msg as std_msgs
import quadrotor_msgs.msg as quadrotor_msgs
from rl.lxx_baselines.ppo.ppo2 import PPO2


class NetworkController:
    def __init__(self):
        rospy.init_node("fly_neural_network")
        self.rate = rospy.Rate(50)

        self._state_estimate_sub = rospy.Subscriber("/hummingbird/ground_truth/odometry", Odometry, self.state_estimate_callback)
        self._control_command_pub = rospy.Publisher("/hummingbird/autopilot/control_command_input", quadrotor_msgs.ControlCommand, queue_size=3)

        self.neural_network = PPO2.load('./NT_12_NegativeThrust.zip')
        
        self.neural_network_obs = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float32)
        self.neural_network_act = None

        self.obs = Odometry()
        self.quat = None
        self.r = None
        self.euler_zyx = None
        self.act = quadrotor_msgs.ControlCommand()

    def state_estimate_callback(self, msg):
        self.obs = msg
        self.quat = np.array([self.obs.pose.pose.orientation.x, self.obs.pose.pose.orientation.y, self.obs.pose.pose.orientation.z, self.obs.pose.pose.orientation.w], dtype=np.float32)
        self.r = R.from_quat(self.quat)
        self.euler_zyx = self.r.as_euler('ZYX', degrees=False)
        self.neural_network_obs = np.array([self.obs.pose.pose.position.x,
                                            self.obs.pose.pose.position.y,
                                            self.obs.pose.pose.position.z - 1.0,
                                            self.euler_zyx[0],
                                            self.euler_zyx[1],
                                            self.euler_zyx[2],
                                            self.obs.twist.twist.linear.x,
                                            self.obs.twist.twist.linear.y,
                                            self.obs.twist.twist.linear.z,
                                            self.obs.twist.twist.angular.x,
                                            self.obs.twist.twist.angular.y,
                                            self.obs.twist.twist.angular.z], dtype=np.float32)
        
    def run(self):
        while not rospy.is_shutdown():
                       
            if (self.neural_network_obs - np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float32)).all == False:
                pass 
            else:
                self.neural_network_act, _ = self.neural_network.predict(self.neural_network_obs, deterministic=True)
                self.act.control_mode = 2
                self.act.armed = True

                self.act.bodyrates.x = self.neural_network_act[0] * 3.1415926 * 2
                self.act.bodyrates.y = self.neural_network_act[1] * 3.1415926 * 2
                self.act.bodyrates.z = self.neural_network_act[2] * 3.1415926 
                self.act.collective_thrust = self.neural_network_act[3] * 9.81 * 2 + 9.81 
                if (self.act.collective_thrust < 0):
                    self.act.collective_thrust = 0

                print("==============================================")
                print(self.act)
                print("==============================================")

                self._control_command_pub.publish(self.act)

            self.rate.sleep()

if __name__ == "__main__":
    network = NetworkController()
    network.run()