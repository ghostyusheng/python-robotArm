import cv2
import imutils
import numpy as np
from imutils import paths


# 获取目标的轮廓信息
def find_marker(image):

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # 颜色空间转换函数，RGB和BGR颜色空间转换 opencv默认的颜色空间是BGR
    gray = cv2.GaussianBlur(gray, (5, 5), 0)  # 高斯滤波，对图像进行滤波操作 ，（5,5）表示高斯核的大小 ，0 表示标准差取0
    edged = cv2.Canny(gray, 35, 125)  # canny 算子 边缘检测 35是阈值1， 125是阈值2，大的阈值用于检测图像中的明显边缘，小的阈值用于将不明显的边缘检测连接起来

    cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)  # 找到详细的轮廓点， RETR_LIST 以列表的形式输出轮廓信息
    # CHAIN_APPROX_SIMPLE： 压缩水平方向，垂直方向，对角线方向的元素，只保留该方向的终点坐标

    cnts = imutils.grab_contours(cnts)  # 寻找图像的轮廓信息，输入图像为一个二值图像

    c = max(cnts, key=cv2.contourArea)  # 轮廓点的面积计算
    # return edged
    return cv2.minAreaRect(c)  # 求出在 C点集下的像素点的面积


def get_F(W, D):

    P = find_marker(image)[1][0] / 118.11  # 300dim  1cm = 118.11像素值

    return (P*D) / W  # F = get_F(2, 126)  # 输出的值为cm f=24.003cm 焦距


def distance_to_camera(F, P, W):

    return (F*W) / P  # F 为相机的焦距，w为物体的宽度，P为物体在照片中的像素宽度，需要转换为cm


W = 2  # 需要手动测量目标的宽度，单位为cm
F = 24.00304  # 根据get_F求出 ,get_F()函数是为了求得相机的焦距，需要通过测试图像中的目标距离来求出

#image = cv2.imread('065_5.jpg')
#marker = find_marker(image)
#P = marker[1][0] / 118.11  # 300dim 1cm = 118.11像素值 ，300dim指300分辨率，有1080分辨率，像素值的㎝转换是不同
#inches = distance_to_camera(F, P, W) #
#print('距离为：%.2f cm' % inches)  # 单位为cm
#
## draw a bounding box around the image and display it
#box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
#box = np.int0(box)
#cv2.drawContours(image, [box], -1, (0, 255, 0), 2)
#cv2.putText(image, "%.2fft" % inches,
#            (image.shape[1] - 200, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
#            2.0, (0, 255, 0), 3)
#cv2.imshow("image", image)
#
#cv2.waitKey(0)
#cv2.destroyAllWindows()
#
