#!/usr/bin/env python
#coding=utf-8

import numpy as np
import os, json
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
from gazebo_msgs.srv import *


def image_callback(data):
    img_pos = Image()
    # define picture to_down' coefficient of ratio
    img_pos.header.stamp = rospy.Time.now()
    global bridge
    
    #the process of the data
    # cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    np_arr = np.fromstring(data.data, np.uint8)
    cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    hue_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
    low_range = np.array([0, 200, 40])   #[0, 230, 80]
    high_range = np.array([10, 255, 210]) # [5, 256, 200]
    th = cv2.inRange(hue_image, low_range, high_range)
    dilated = cv2.dilate(th, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)

    # # 是否显示图像
    # cv2.imshow("img", cv_img)
    # cv2.imshow("hsv", dilated)
    # cv2.waitKey(1)

    M = cv2.moments(dilated, binaryImage = True)
    if M["m00"] != 0:
        # img_pos.header.stamp = rospy.Time.now()
        img_pos.width = int(M["m10"] / M["m00"])
        img_pos.height = int(M["m01"] / M["m00"])
    else:
        # img_pos.header.stamp = rospy.Time.now()
        img_pos.width = 0
        img_pos.height = 0

    imag_pub.publish(img_pos)


if __name__ == '__main__':
    global imag_pub
    imag_pub = rospy.Publisher("tracker/pos_image", Image, queue_size=10)  # 发送图像位置
    rospy.init_node('iris_fpv_cam', anonymous=True)

    global bridge
    bridge = CvBridge()
    # rospy.Subscriber('/camera/left/compressed', Image, image_callback)
    rospy.Subscriber('/camera/left/compressed', CompressedImage, image_callback)

    rospy.spin()
