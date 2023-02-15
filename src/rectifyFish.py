# -*- coding: utf-8 -*-
"""
Created on Mon Aug 15 15:15:58 2022.

@author: xiaoqingtech01
"""

import cv2, tf
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge

rospy.init_node('rectifyFish', anonymous=True)  # 定义节点
leftPub = rospy.Publisher('/camera1/cam/rect', Image, queue_size=30)  # 定义话题
rightPub = rospy.Publisher('/camera2/cam/rect', Image, queue_size=30)  # 定义话题
rate = rospy.Rate(30)


def rectifyLeft(data):
    global bridge, map1x, map1y, frameRate
    image_raw = bridge.imgmsg_to_cv2(data, "bgr8")

    ros_frame = Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = "rectifyFish"
    ros_frame.header = header
    ros_frame.width = 3000  # 尺寸与opencv图像一致
    ros_frame.height = 3000
    ros_frame.encoding = "bgr8"
    ros_frame.step = 3000 * 3
    rect = cv2.remap(image_raw, map1x, map1y, cv2.INTER_LINEAR,
                     cv2.BORDER_CONSTANT)
    ros_frame.data = np.array(rect).tostring()  # 图片格式转换
    leftPub.publish(ros_frame)  # 发布消息
    br.sendTransform((0, 0, 0),
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),"rectifyFish","world") 


def rectifyRight(data):
    global bridge, map1x, map1y
    image_raw = bridge.imgmsg_to_cv2(data, "bgr8")

    ros_frame = Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = "rectifyFish"
    ros_frame.header = header
    ros_frame.width = 3000  # 尺寸与opencv图像一致
    ros_frame.height = 3000
    ros_frame.encoding = "bgr8"
    ros_frame.step = 3000 * 3
    rect = cv2.remap(image_raw, map1x, map1y, cv2.INTER_LINEAR,
                     cv2.BORDER_CONSTANT)
    ros_frame.data = np.array(rect).tostring()  # 图片格式转换
    rightPub.publish(ros_frame)  # 发布消息


def displayWebcam():
    global bridge, map1x, map1y, frameRate, br
    frameRate = 0
    br = tf.TransformBroadcaster()
    map1x, map1y = np.load('map140x.npy'), np.load('map140y.npy')
    bridge = CvBridge()
    rospy.Subscriber('/camera1/cam/image_raw', Image, rectifyLeft)
    rospy.Subscriber('/camera2/cam/image_raw', Image, rectifyRight)
    rospy.spin()


if __name__ == '__main__':
    displayWebcam()
