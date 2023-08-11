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
def cap():
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
    print("Camera model: ", cam.get_model_name())
    print("Vendor Name: ", cam.get_vendor_name())
    print("Device id: ", cam.get_device_id())
    print("Region: ", cam.get_region())
    try:
       cam.start_acquisition_continuous()
       print("Camera On")
       #cv2.namedWindow('capture', flags=0)
       count = 0
       while True:
           count += 1
           frame = cam.pop_frame()
           print("[", time.time(), "] frame nb: ", count, " shape: ", frame.shape)
           if not 0 in frame.shape:
              image = cv2.cvtColor(frame, cv2.COLOR_BayerRG2RGB)
              path = "image.png"
              #cv2.imshow("capture", image)
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

def gclose():
    with eva.lock():
       eva.control_wait_for_ready()
       eva.gpio_set('ee_d0', False)
       eva.gpio_set('ee_d1', True)

def gopen():
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
gopen()

time.sleep(3)
cap()

time.sleep(1)

print('close')
gclose()

time.sleep(1)

with eva.lock():
    eva.control_wait_for_ready()
    eva.control_go_to([-0.9599310885968813,
     -0.3490658503988659,
     -1.5707963267948966,
     0,
     -1.2217304763960306,
     0.0])

gopen()
show()
#
#toolpaths = eva.toolpaths_list()
#outToolpaths = []
#for toolpathItem in toolpaths:
#  toolpath = eva.toolpaths_retrieve(toolpathItem['id'])
#  outToolpaths.append(toolpath)
#print(dumps(outToolpaths))
#toolpath = {
#  "metadata": {
#      "version": 2,
#      "payload": 0,
#      "default_max_speed": 1.05,
#      "next_label_id": 5,
#      "analog_modes": {"i0": "voltage", "i1": "voltage", "o0": "voltage", "o1": "voltage"},
#  },
#  "waypoints": [
#      {"joints": [-0.68147224, 0.3648368, -1.0703622, 9.354615e-05, -2.4358354, -0.6813218], "label_id": 3},
#      {"joints": [-0.6350288, 0.25192022, -1.0664424, 0.030407501, -2.2955494, -0.615318], "label_id": 2},
#      {"joints": [-0.13414459, 0.5361486, -1.280493, -6.992453e-08, -2.3972468, -0.13414553], "label_id": 1},
#      {"joints": [-0.4103904, 0.33332264, -1.5417944, -5.380291e-06, -1.9328799, -0.41031334], "label_id": 4},
#  ],
#  "timeline": [
#      {"type": "home", "waypoint_id": 2},
#      {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 1},
#      {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 0},
#      {"type": "trajectory", "trajectory": "joint_space", "waypoint_id": 2},
#  ],
#}




