#!/usr/bin/env python3

import json

from numpy import arange
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ManageSpeed:

    def __init__(self):
        self.cam_info_sub = rospy.Subscriber("/dronecv/image/info", String, self.calc_speed_callback)
        self.pub_vel = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        self.gain_p = 1.3
        self.gain_i = 0.1
        self.gain_d = 0.1
        self.error_last = 0
        self.error_accumulated = 0
        self.linear_speed_factor = 200
        self.angular_speed_factor = -0.0010


    def calc_speed_callback(self, message):
        payload = json.loads(message.data)
        area, error = payload["Area"], payload["Erro"]
        print(area, error)
        error = self.controller_pid(error)

        velocity_message = Twist()

        if (area>4000):
            velocity_message.linear.x = self.linear_speed_factor/area
            Az = error * self.angular_speed_factor
            print('max_c_area= ', area)
            if abs(Az)>0.1:
                velocity_message.angular.z = Az
                print('Turning speed ', velocity_message.angular.z)
            else:
                velocity_message.angular.z =0 
          
            self.pub_vel.publish(velocity_message)

        else:
            velocity_message.linear.x=0
            velocity_message.angular.z=0
            self.pub_vel.publish(velocity_message)



    def controller_pid(self, error):
        error_diference = error - self.error_last
        self.error_last = error
        self.error_accumulated += error
        return - error * self.gain_p - error_diference * self.gain_d - self.error_accumulated * self.gain_i

if __name__ == '__main__':
    try:
        rospy.init_node("manage_speed_node")
        ManageSpeed()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass