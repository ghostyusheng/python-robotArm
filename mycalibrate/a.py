import cv2
import time

# 图片保存路径
IMG_SAVE_PATH = "img/"

num = 1

def run():
    global num
    camera = cv2.VideoCapture(0)
    state, src = camera.read()
    print(src.shape)
    #cv2.imshow('src', src)
    #cv2.imwrite(IMG_SAVE_PATH + str(num) + '.jpg', src)
    #print("Saved img_" + str(num) + "!")
    #num +=1


if __name__ == '__main__':
    while True:
        run()
        time.sleep(3)
    cv2.destroyAllWindows()
