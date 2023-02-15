from sys import set_coroutine_origin_tracking_depth
import numpy as np
import cv2, tf
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge
import os, pickle


rospy.init_node('diffFIsh', anonymous=True)  # 定义节点
leftPub = rospy.Publisher('/camera1/cam/rect', Image, queue_size=30)  # 定义话题
rightPub = rospy.Publisher('/camera2/cam/rect', Image, queue_size=30)  # 定义话题
rate = rospy.Rate(30)



def rectifyLeft(data):
    global bridge, map1x, map1y, rectL, rectR
    image_raw = bridge.imgmsg_to_cv2(data, "bgr8")
    # image_raw = cv2.imread('../north.jpg')
    # M = cv2.getRotationMatrix2D((1500, 1500), 86+15, 1)
    # image_raw = cv2.warpAffine(src=image_raw, M=M, dsize=(3000, 3000),
    #                         borderValue=(0, 0, 0))
    if not os.path.exists('/home/ubuntu/Desktop/csa_cloudhight/calibration/left.jpg'):
        cv2.imwrite('/home/ubuntu/Desktop/csa_cloudhight/calibration/left.jpg', image_raw)

    if not os.path.exists('image.png'):
        cv2.imwrite('image.png', image_raw)

    rectL = cv2.remap(image_raw, map1x, map1y, cv2.INTER_LINEAR,
                     cv2.BORDER_CONSTANT)
                     
    ros_frame = Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = "rectifyFish"
    ros_frame.header = header
    ros_frame.width = 3000  # 尺寸与opencv图像一致
    ros_frame.height = 3000
    ros_frame.encoding = "bgr8"
    ros_frame.step = 3000 * 3
    
    ros_frame.data = np.array(rectL).tostring()  # 图片格式转换
    leftPub.publish(ros_frame)  # 发布消息
    br.sendTransform((0, 0, 0),
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),"rectifyFish","world") 
    

def rectifyRight(data):
    global bridge, map2x, map2y, rectL, rectR
    image_raw = bridge.imgmsg_to_cv2(data, "bgr8")
    # image_raw = cv2.imread('../center.jpg')

    # M = cv2.getRotationMatrix2D((1500, 1500), 86, 1)
    # image_raw = cv2.warpAffine(src=image_raw, M=M, dsize=(3000, 3000),
    #                         borderValue=(0, 0, 0))
    if not os.path.exists('/home/ubuntu/Desktop/csa_cloudhight/calibration/right.jpg'):
        cv2.imwrite('/home/ubuntu/Desktop/csa_cloudhight/calibration/right.jpg', image_raw)

    rectR = cv2.remap(image_raw, map2x, map2y, cv2.INTER_LINEAR,
                     cv2.BORDER_CONSTANT)

    
    ros_frame = Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = "rectifyFish"
    ros_frame.header = header
    ros_frame.width = 3000  # 尺寸与opencv图像一致
    ros_frame.height = 3000
    ros_frame.encoding = "bgr8"
    ros_frame.step = 3000 * 3
    ros_frame.data = np.array(rectR).tostring()  # 图片格式转换
    rightPub.publish(ros_frame)  # 发布消息

    # update()
    # compute()

def update(val=1):
    global stereo
    stereo.setBlockSize(cv2.getTrackbarPos('window_size', 'disparity'))
    stereo.setUniquenessRatio(cv2.getTrackbarPos('uniquenessRatio', 'disparity'))
    stereo.setSpeckleWindowSize(cv2.getTrackbarPos('speckleWindowSize', 'disparity'))
    stereo.setSpeckleRange(cv2.getTrackbarPos('speckleRange', 'disparity'))
    stereo.setDisp12MaxDiff(cv2.getTrackbarPos('disp12MaxDiff', 'disparity'))
    
    if cv2.waitKey(1) == 27:
        rospy.signal_shutdown("closed!")


def compute():
    global rectL, rectR
    print('computing disparity...')
    # rectL = cv2.resize(rectL,dsize=None,fx=0.2,fy=0.2)
    # rectR = cv2.resize(rectR,dsize=None,fx=0.2,fy=0.2)
    disp = stereo.compute(rectL, rectR).astype(np.float32) / 16.0

    cv2.imshow('comp', cv2.resize(cv2.normalize(disp, disp, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U),(1000,1000)))
    if cv2.waitKey(1) == 27:
        rospy.signal_shutdown("closed!")

    # Q = np.array([[ 1.00000000e+00 , 0.00000000e+00 , 0.00000000e+00, -1.53116940e+03],
    #                 [ 0.00000000e+00  ,1.00000000e+00 , 0.00000000e+00 ,-1.41097534e+03],
    #                 [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 9.96697158e+02],
    #                 [ 0.00000000e+00 , 0.00000000e+00 , 5.81395349e-07 ,-1.32956571e-05]],dtype=np.float32)
    Q = np.array([[ 1.00000000e+00 , 0.00000000e+00 , 0.00000000e+00, -1.53116940e+03],
                    [ 0.00000000e+00  ,1.00000000e+00 , 0.00000000e+00 ,-1.41097534e+03],
                    [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 9.96697158e+02],
                    [ 0.00000000e+00 , 0.00000000e+00 , 1/150000 ,-163.34664404473526/150000]],dtype=np.float32)
    if not os.path.exists('pts.npy'):
        pts = cv2.reprojectImageTo3D(disp, Q)
        np.save('pts.npy',pts)
    
    

if __name__ == "__main__":
    window_size = 5
    min_disp = 16
    num_disp = 192-min_disp
    blockSize = window_size
    uniquenessRatio = 1
    speckleRange = 3
    speckleWindowSize = 3
    disp12MaxDiff = 200
    P1 = 48*window_size
    P2 = 172*window_size
    scale = 1
    br = tf.TransformBroadcaster()
    # imgL = cv2.imread(r'left.jpg')
    # imgR = cv2.imread(r'right.jpg')
    map1x, map1y = np.load('../mat/map140x.npy'), np.load('../mat/map140y.npy')
    map2x, map2y = map1x.copy(), map1y.copy()
    # map1x, map1y = np.load('../mat/t265leftx.npy'), np.load('../mat/t265lefty.npy')
    # map2x, map2y = np.load('../mat/t265rightx.npy'), np.load('../mat/t265righty.npy')
    # imgL = cv2.remap(imgL, map1x, map1y, cv2.INTER_LINEAR,
    #                  cv2.BORDER_CONSTANT)
    # imgR = cv2.remap(imgR, map1x, map1y, cv2.INTER_LINEAR,
    #                  cv2.BORDER_CONSTANT)
    bridge = CvBridge()
    rospy.Subscriber('/camera1/cam/image_raw', Image, rectifyLeft)
    rospy.Subscriber('/camera2/cam/image_raw', Image, rectifyRight)
    # rospy.Subscriber('/T265_camera/fisheye1/image_raw', Image, rectifyLeft)
    # rospy.Subscriber('/T265_camera/fisheye2/image_raw', Image, rectifyRight)
    # rectL = cv2.resize(imgL,dsize=None,fx=0.2,fy=0.2)
    # rectR = cv2.resize(imgR,dsize=None,fx=0.2,fy=0.2)

    cv2.namedWindow('disparity')
    cv2.createTrackbar('speckleRange', 'disparity', speckleRange, 50, update)    
    cv2.createTrackbar('window_size', 'disparity', window_size, 50, update)
    cv2.createTrackbar('speckleWindowSize', 'disparity', speckleWindowSize, 200, update)
    cv2.createTrackbar('uniquenessRatio', 'disparity', uniquenessRatio, 15, update)
    cv2.createTrackbar('disp12MaxDiff', 'disparity', disp12MaxDiff, 250, update)
    stereo = cv2.StereoSGBM_create(
        minDisparity = min_disp*scale,  #表示可能的最小视差值。通常为0，但有时校正算法会移动图像，所以参数值也要相应调整
        numDisparities = num_disp*scale, #表示最大的视差值与最小的视差值之差，这个差值总是大于0。在当前的实现中，这个值必须要能被16整除
        blockSize = window_size*scale,   
        uniquenessRatio = uniquenessRatio,#表示由代价函数计算得到的最好（最小）结果值比第二好的值小多少（用百分比表示）才被认为是正确的。通常在5-15之间。
        speckleRange = speckleRange*scale,   #指每个已连接部分的最大视差变化，如果进行斑点过滤，则该参数取正值，函数会自动乘以16、一般情况下取1或2就足够了。
        speckleWindowSize = speckleWindowSize*scale,  #表示平滑视差区域的最大窗口尺寸，以考虑噪声斑点或无效性。将它设为0就不会进行斑点过滤，否则应取50-200之间的某个值。
        disp12MaxDiff = disp12MaxDiff,  #表示在左右视图检查中最大允许的偏差（整数像素单位）。设为非正值将不做检查。
        P1 = P1,  #控制视差图平滑度的第一个参数
        P2 = P2   #控制视差图平滑度的第二个参数，值越大，视差图越平滑。P1是邻近像素间视差值变化为1时的惩罚值，
                  #p2是邻近像素间视差值变化大于1时的惩罚值。算法要求P2>P1,stereo_match.cpp样例中给出一些p1和p2的合理取值。
    )
    rospy.spin()
