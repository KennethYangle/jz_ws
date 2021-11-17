#!/usr/bin/env python
#coding=utf-8

import numpy as np
import os, json
import rospy, rospkg
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
from swarm_msgs.msg import BoundingBox, BoundingBoxes


def image_callback(data):
    global is_show_rgb, is_show_hsv
    img_pos = BoundingBoxes()
    # define picture to_down' coefficient of ratio
    img_pos.header.stamp = rospy.Time.now()
    global bridge
    
    #the process of the data
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    # np_arr = np.fromstring(data.data, np.uint8)
    # cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    hue_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
    low_range1 = np.array([175, 200, 40])   #[0, 230, 80]
    high_range1 = np.array([180, 256, 210]) # [5, 256, 200]
    th1 = cv2.inRange(hue_image, low_range1, high_range1)
    low_range2 = np.array([0, 200, 40])   #[0, 230, 80]
    high_range2 = np.array([10, 256, 210]) # [5, 256, 200]
    th2 = cv2.inRange(hue_image, low_range2, high_range2)
    th = th1 + th2
    dilated = cv2.dilate(th, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)

    # 连通域分析, https://stackoverflow.com/questions/35854197/how-to-use-opencvs-connected-components-with-stats-in-python/35854198#35854198
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(dilated)
    id = 0
    for i in range(1, num_labels):          # 0是背景
        left = stats[i, cv2.CC_STAT_LEFT]
        top  = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        area = stats[i, cv2.CC_STAT_AREA]

        if area > 50:
            bbox = BoundingBox()
            bbox.probability = 1
            bbox.xmin = left
            bbox.ymin = top
            bbox.xmax = left+w
            bbox.ymax = top+h
            bbox.id = id
            img_pos.bounding_boxes.append(bbox)
            id += 1

            cv2.rectangle(cv_img, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (0, 255, 0), 2)

    # 是否显示图像
    if is_show_rgb:
        cv2.imshow("img", cv_img)
    if is_show_hsv:
        cv2.imshow("hsv", dilated)
    cv2.waitKey(1)

    imag_pub.publish(img_pos)


if __name__ == '__main__':
    src_path = os.path.join(rospkg.RosPack().get_path("offboard_pkg"), "..")
    setting_file = open(os.path.join(src_path, "settings.json"))
    setting = json.load(setting_file)
    print(json.dumps(setting, indent=4))

    is_show_rgb = setting["Debug"]["is_show_rgb"]
    is_show_hsv = setting["Debug"]["is_show_hsv"]
    
    global imag_pub
    rospy.init_node('iris_fpv_cam', anonymous=True)

    global bridge
    bridge = CvBridge()
    rospy.Subscriber('/camera/rgb', Image, image_callback)
    # rospy.Subscriber('/camera/rgb/compressed', CompressedImage, image_callback)
    imag_pub = rospy.Publisher("tracker/pos_image", BoundingBoxes, queue_size=10)  # 发送图像位置

    rospy.spin()
