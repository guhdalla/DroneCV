#!/usr/bin/env python3
import rospy
from controller.listener_cam import ListenerCam
from enums.enum_mask_color import EnumColorMask

class DroneCV:
    
    def __init__(self, color):
        self.video = ListenerCam(color)

    
if __name__ == '__main__':
    try:
        rospy.init_node("dronecv_node")
        drone = DroneCV(EnumColorMask.GREEN.value)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
