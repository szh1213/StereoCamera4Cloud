# coding: utf-8
import cv2
import numpy as np
import matplotlib.pyplot as plt
img = cv2.imread("image.png")


# print img.shape
pts=-np.load('pts.npy')
plt.imshow(pts[:,:,:])
plt.show()
def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        x=x*2
        y=y*2
        pos = str(pts[y//5,x//5,2]//1000)
        cv2.circle(img, (x, y), 1, (255, 0, 0), thickness=-1)
        cv2.putText(img, pos, (x, y), cv2.FONT_HERSHEY_PLAIN,
                    5.0, (0, 0, 0), thickness=3)
        cv2.imshow("image", cv2.resize(img,(1500,1500)))


cv2.namedWindow("image")
cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
cv2.imshow("image",cv2.resize(img,(1500,1500)))

while (True):
    try:
        cv2.waitKey(100)
    except Exception:
        cv2.destroyAllWindows()
        break

cv2.waitKey(0)
cv2.destroyAllWindows()