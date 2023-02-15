#!/usr/bin/env python
#!coding=utf-8

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


def saveLeft(data):
    global bridge, leftFrameCount
    if leftFrameCount > 0:
        return
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")

    image_name = "left.jpg"  # 图像命名：时间戳.jpg
    cv2.imwrite(image_name, cv_img)  # 保存；
    leftFrameCount += 1
    print('saved left.jpg')


def saveRight(data):
    global bridge, rightFrameCount
    if rightFrameCount > 0:
        return
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")

    image_name = "right.jpg"  # 图像命名：时间戳.jpg
    cv2.imwrite(image_name, cv_img)  # 保存；
    rightFrameCount += 1
    print('saved right.jpg')
    exit()


def displayWebcam():
    rospy.init_node('webcam_display', anonymous=True)

    # make a video_object and init the video object
    global bridge, leftFrameCount, rightFrameCount
    leftFrameCount, rightFrameCount = 0, 0
    bridge = CvBridge()
    rospy.Subscriber('/camera1/cam/image_raw', Image, saveLeft)
    rospy.Subscriber('/camera2/cam/image_raw', Image, saveRight)
    rospy.spin()


if __name__ == '__main__':
    displayWebcam()
