import time
import cv2
from aravis import Camera

camera_hostname = "evacctv03"
camera_ip = "144.32.152.11"
cam = Camera()
cam.set_feature("Width", 1936)
cam.set_feature("Height", 1216)
cam.set_frame_rate(10)
cam.set_exposure_time(100000)
cam.set_pixel_format_from_string('BayerRG8')

print("Camera model: ", cam.get_model_name())
print("Vendor Name: ", cam.get_vendor_name())
print("Device id: ", cam.get_device_id())
print("Region: ", cam.get_region())

cam.start_acquisition_continuous()
cv2.namedWindow('capture', flags=0)
image = cv2.cvtColor(frame, cv2.COLOR_BayerRG2RGB)
cv2.imshow("capture", image)
cv2.waitKey(1)
dp = Matrix([0, -0.2, 0, 0, 0, 0])
ans = jacobian_move_to_1_then_2(ans[0], ans[1], ans[2], ans[3], ans[4], ans[5], dp,0)

dp = Matrix([-0.2, 0, 0, 0, 0, 0])
ans = jacobian_move_to_1_then_2(ans[0], ans[1], ans[2], ans[3], ans[4], ans[5], dp,0)
dp = Matrix([0, 0.2, 0, 0, 0, 0])
ans = jacobian_move_to_1_then_2(ans[0], ans[1], ans[2], ans[3], ans[4], ans[5], dp,0)

