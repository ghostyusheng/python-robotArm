from evasdk import Eva
import time
import math
from json import dumps
import time
import cv2
from aravis import Camera
from myimg import show
import argparse
import imutils
import cv2
def capture():
    #camera_hostname = "evacctv02"
    #camera_ip = "144.32.152.10"
    #cam = Camera('S1188411')
    camera_hostname = "evacctv03"
    camera_ip = "144.32.152.11"
    cam = Camera('S1188413')
    cam.set_feature("Width", 1936)
    cam.set_feature("Height", 1216)
    cam.set_frame_rate(10)
    cam.set_exposure_time(100000)
    cam.set_pixel_format_from_string('BayerRG8')
    try:
       cam.start_acquisition_continuous()
       count = 0
       while True:
           count += 1
           frame = cam.pop_frame()
           print("[", time.time(), "] frame nb: ", count, " shape: ", frame.shape)
           if not 0 in frame.shape:
              image = cv2.cvtColor(frame, cv2.COLOR_BayerRG2RGB)
              path = "image.png"
              cv2.imwrite(path, image)
              break
              cv2.waitKey(1)
    except KeyboardInterrupt:
       print("Exiting...")
    finally:
       cam.stop_acquisition()
       cam.shutdown()
       print("Camera Off")

arm_hostname = "evacunningyorkartistpt410"
arm_ip = "144.32.152.2"
token = "4b1c26278a566e0419165a3acd025dd83d32b160"

def dclose():
    with eva.lock():
       eva.control_wait_for_ready()
       eva.gpio_set('ee_d0', False)
       eva.gpio_set('ee_d1', True)

def dopen():
    with eva.lock():
       eva.control_wait_for_ready()
       eva.gpio_set('ee_d0', True)
       eva.gpio_set('ee_d1', False)

eva = Eva(arm_ip, token)

with eva.lock():
    eva.control_wait_for_ready()
    eva.control_go_to([-0.9599310885968813,
     -0.3490658503988659,
     -1.5707963267948966,
     0,
     -1.2217304763960306,
     0.0])

with eva.lock():
    eva.control_wait_for_ready()
    eva.control_go_to([0.0, -0.6108652381980153, -1.5707963267948966, 0, -0.9424777960769379, 0.0])

print('cap')
dopen()

time.sleep(3)
capture()

time.sleep(1)

print('close')
dclose()

time.sleep(1)

with eva.lock():
    eva.control_wait_for_ready()
    eva.control_go_to([-0.9599310885968813,
     -0.3490658503988659,
     -1.5707963267948966,
     0,
     -1.2217304763960306,
     0.0])

dopen()
show()
