#!/usr/bin/env python
#!coding=utf-8
 
#right code !
#write by leo at 2018.04.26
#function: 
#display the frame from another node.
 
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import message_filters
from gazebo_msgs.srv import *


imageWidth = 640
imageHeight = 480
imageSize = (imageWidth, imageHeight)

'''左目相机标定参数
fc_left_x   0            cc_left_x
0           fc_left_y    cc_left_y
0           0            1
'''
cameraMatrixL = np.array([[1034.4730060050647, 0.0, 320.5],
                          [0.0, 1034.4730060050647,240.5],
                          [0.0, 0.0, 1.0]])

# [kc_left_01,  kc_left_02,  kc_left_03,  kc_left_04,   kc_left_05]
distCoeffL = np.array([0,0,0,0,0])

'''右目相机标定参数
fc_right_x   0              cc_right_x
0            fc_right_y     cc_right_y
0            0              1
'''
cameraMatrixR = np.array([[1034.4730060050647, 0.0, 320.5],
                          [0.0, 1034.4730060050647,240.5],
                          [0.0, 0.0, 1.0]])

# kc_right_01,  kc_right_02,  kc_right_03,  kc_right_04,   kc_right_05
distCoeffR = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
model = GetModelStateRequest()
model.model_name = 'camera2'

def callback(data1,data2):
    global model
    frame1 = bridge1.imgmsg_to_cv2(data1, "bgr8")
    frame2 = bridge2.imgmsg_to_cv2(data2, "bgr8")
    objstate = get_state_service(model)
    state = (objstate.pose.position.x, objstate.pose.position.y, objstate.pose.position.z)

    # T平移向量
    T = np.array([52,0.0,0.0])

    # rec旋转向量
    rec = np.array([-0.00, -0.00, -0.000])

    # 立体校正
    R = cv2.Rodrigues(rec)[0]
    Rl, Rr, Pl, Pr, Q, validROIL, validROIR = cv2.stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR,
                                                                imageSize, R, T,)

    # 计算更正map
    mapLx, mapLy = cv2.initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, cv2.CV_32FC1)
    mapRx, mapRy = cv2.initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, cv2.CV_32FC1)

    # 读取图片
    rgbImageL = frame1
    grayImageL = cv2.cvtColor(rgbImageL, cv2.COLOR_BGR2GRAY)
    rgbImageR = frame2
    grayImageR = cv2.cvtColor(rgbImageR, cv2.COLOR_BGR2GRAY)

    # 经过remap之后，左右相机的图像已经共面并且行对齐
    rectifyImageL = cv2.remap(grayImageL, mapLx, mapLy, cv2.INTER_LINEAR)
    rectifyImageR = cv2.remap(grayImageR, mapRx, mapRy, cv2.INTER_LINEAR)

    # SGBM算法重要的参数
    mindisparity = 32
    SADWindowSize = 16
    ndisparities = 176
    # 惩罚系数
    P1 = 4 * 1 * SADWindowSize * SADWindowSize
    P2 = 32 * 1 * SADWindowSize * SADWindowSize

    # BM算法
    sgbm = cv2.StereoSGBM_create(mindisparity, ndisparities, SADWindowSize)
    sgbm.setP1(P1)
    sgbm.setP2(P2)

    sgbm.setPreFilterCap(60)
    sgbm.setUniquenessRatio(30)
    sgbm.setSpeckleRange(2)
    sgbm.setSpeckleWindowSize(200)
    sgbm.setDisp12MaxDiff(1)
    disp = sgbm.compute(rectifyImageL, rectifyImageR)

    # 在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16
    xyz = cv2.reprojectImageTo3D(disp, Q, handleMissingValues=True)
    xyz = xyz * 16

    # 用于显示处理
    disp = disp.astype(np.float32) / 16.0
    disp8U = cv2.normalize(disp, disp, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    disp8U = cv2.medianBlur(disp8U, 9)
    
        
    # 鼠标点击事件
    def onMouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print('点 (%d, %d) 的三维坐标 (%f, %f, %f)' % (x, y, xyz[y, x, 0], xyz[y, x, 1], xyz[y, x, 2]))

    # 显示图片
    cv2.imshow("disparity", disp8U)
    cv2.setMouseCallback("disparity", onMouse, 0)


    cv2.imshow("camera1" , rectifyImageL)
    cv2.imshow("camera2" , rectifyImageR)
    cv2.waitKey(3)

 
def display():
    rospy.init_node('webcam_puber', anonymous=True)
    # make a video_object and init the video object
    global count1,bridge1,frame1,count2,bridge2,frame2
    count1,count2 = 0,0
    bridge1 = CvBridge()
    
    bridge2 = CvBridge()
   

    camera1 = message_filters.Subscriber('camera1/camera1/img',Image)
    camera2 = message_filters.Subscriber('camera2/camera2/img',Image)
    ts = message_filters.TimeSynchronizer([camera1, camera2], 10)
    ts.registerCallback(callback)
    rospy.spin()

 
if __name__ == '__main__':
    display()
 
