import cv2
import numpy as np

def show():
    img = cv2.imread("./image.png")
    cv2.imshow("Image", img)
    cv2.waitKey (0)
    cv2.destroyAllWindows()
