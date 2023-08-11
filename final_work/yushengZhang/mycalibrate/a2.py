import cv2
import time
from aravis import Camera


# 图片保存路径
IMG_SAVE_PATH = "img/"

num = 1
camera_hostname = "evacctv03"
camera_ip = "144.32.152.11"
cam = Camera("S1188413")
cam.set_feature("Width", 1936)
cam.set_feature("Height", 1216)
cam.set_frame_rate(30)
cam.set_exposure_time(100000)
cam.set_pixel_format_from_string('BayerRG8')
print("Camera model: ", cam.get_model_name())
print("Vendor Name: ", cam.get_vendor_name())
print("Device id: ", cam.get_device_id())
print("Region: ", cam.get_region())

def run():
    global num
    camera = cv2.VideoCapture(0)
    state, src = camera.read()
    #cv2.imshow('src', src)
    cv2.imwrite(IMG_SAVE_PATH + str(num) + '.jpg', src)
    print("Saved img_" + str(num) + "!")
    num +=1


if __name__ == '__main__':
    try:
       cam.start_acquisition_continuous()
       print("Camera On")
       cv2.namedWindow('win', flags=0)
       count = 0
       while True:
           count += 1
           frame = cam.pop_frame()
           print("[", time.time(), "] frame nb: ", count, " shape: ", frame.shape)
           if not 0 in frame.shape:
              img = cv2.cvtColor(frame, cv2.COLOR_BayerRG2RGB)
              print(img.shape)
    except Exception as ex:
        cv2.destroyAllWindows()
        print(ex)
