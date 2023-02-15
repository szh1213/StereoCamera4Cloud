#!/usr/bin/env python
# coding:utf-8

import cv2
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge , CvBridgeError
import time

if __name__=="__main__":
    cap = cv2.VideoCapture("rtsp://admin:xiaoqing123@192.168.1.38/Streaming/Channels/2")
    ret = cap.isOpened()
    fps = cap.get(5)/10000
    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break
    
    rospy.init_node('camera_node', anonymous=True) #定义节点
    image_pub=rospy.Publisher('/fish/img', Image, queue_size = 1) #定义话题
    bridge = CvBridge()
    rate = rospy.Rate(20) # 10hz 
    while ret:    # Ctrl C正常退出，如果异常退出会报错device busy！
        ret, frame = cap.read()
        tstep = cap.get(1)
        iloop=fps/2
        while iloop:
            cap.grab()
            iloop -= 1
            if iloop<1:
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break 
            break
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break 
        show = cv2.resize(frame,(4000,3000))
        #cv2.imshow('fr',show[:500,:1500,:])
        
        msg = bridge.cv2_to_imgmsg(show, encoding="bgr8")
        image_pub.publish(msg) #发布消息
        
        #rate.sleep()
    
    cap.release()
    cv2.destroyAllWindows() 
    print("quit successfully!")

