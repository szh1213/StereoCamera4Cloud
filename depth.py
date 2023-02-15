import cv2
import numpy as np

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
distCoeffR = np.array([0,0,0,0,0])

# T平移向量
T = np.array([-59.32102, 0.27563, -0.79807])

# rec旋转向量
rec = np.array([-0.00927, -0.00228, -0.00070])


# 立体校正
R = cv2.Rodrigues(rec)[0]
Rl, Rr, Pl, Pr, Q, validROIL, validROIR = cv2.stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR,
                                                            imageSize, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
                                                            newImageSize=imageSize)

# 计算更正map
mapLx, mapLy = cv2.initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, cv2.CV_32FC1)
mapRx, mapRy = cv2.initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, cv2.CV_32FC1)

# 读取图片
rgbImageL = cv2.imread("img1.png")
grayImageL = cv2.cvtColor(rgbImageL, cv2.COLOR_BGR2GRAY)
rgbImageR = cv2.imread("img2.png")
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

cv2.waitKey(0)
cv2.destroyAllWindows()
