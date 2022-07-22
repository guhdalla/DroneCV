#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from math import dist
from enums.enum_mask_color import EnumColorMask
import json

class ListenerCam:

    def __init__(self):
        self.color = EnumColorMask.GREEN.value
        self.cam_sub = rospy.Subscriber('/bebop2/camera_base/image_raw', Image, self.image_callback)
        self.cam_pub = rospy.Publisher("/dronecv/image", Image, queue_size=10)
        self.cam_info_pub = rospy.Publisher("/dronecv/image/info", String, queue_size=10)
        self.bridge = CvBridge()
        self.is_show = True
        

    def image_callback(self, message):
        frame = self.bridge.imgmsg_to_cv2(message, "bgr8")
        area, img, distance_2_center = self.treat_image(frame)
        self.image_pub(img)
        self.image_info_pub(area, distance_2_center)
        if(self.is_show):
            self.show(img)

    def show(self, img):
        cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
        cv2.imshow("Image", img)

        if cv2.waitKey(30) & 0xff == 27:
            self.is_show = False
            cv2.destroyAllWindows()

    def image_info_pub(self, area, distance_2_center):
        msg = String()
        info = json.dumps(dict(Area = area, Distance = distance_2_center))
        msg.data = info
        self.cam_info_pub.publish(msg)

    def image_pub(self, img):
        self.cam_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

    def treat_image(self, img):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_copy = img.copy()
        mask = cv2.inRange(img_hsv, self.color["L"], self.color["U"])
        mask = cv2.erode(mask, None, iterations=5)
        mask = cv2.dilate(mask, None, iterations=5)
        contours = self.find_contours(mask)
        center = self.find_center(contours, mask)
        area = self.find_area(contours)
        distance_2_center = 0
        if area > 3000:
            self.draw_center(img_copy, center, 10, [0, 255, 0])
            self.draw_contours(img_copy, contours, 3, (0, 255, 0))
            distance_2_center = self.distance_center(center, img_copy) 
        return area, img_copy, distance_2_center

    def find_area(self, contours):
        area = 0
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
        return area

    def find_center(self, contours, mask):
        cx = 0
        cy = 0
        if len(contours) > 0:
            cnt = contours[0]
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        return (cx, cy)

    def draw_center(self, img, center, size, color):
        cX, cY = center
        cv2.line(img, (cX - size, cY), (cX + size, cY), color, 3)
        cv2.line(img, (cX, cY - size), (cX, cY + size), color, 3)

    def find_contours(self, mask):
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def draw_contours(self, img, contours, size, color):
        cv2.drawContours(img, contours, -1, color, size)

if __name__ == '__main__':
    try:
        rospy.init_node("listener_cam_node")
        ListenerCam()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass