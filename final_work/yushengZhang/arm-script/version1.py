import time
import cv2
import math
import threading
import imutils
import math
import numpy
import matplotlib.pyplot
import sympy
import pickle
from shape import Shape
from evasdk import Eva
from remote import send
from mpl_toolkits.mplot3d import Axes3D
from sympy import Matrix, Symbol, symbols, solveset, solve, simplify, S, diff, det, erf, log, sqrt, pi, sin, cos, tan, \
from aravis import Camera
from evasdk import Eva, asin, acos, atan2, init_printing, pretty_print

pxl = 90.5
rArm = True
C_x, C_y = 965, 910
inches = 0
r_distance = 0
obj_x = 0
obj_y = 0
deltaX = 0
deltaY = 0
log = {}

arm_hostname = "evacunningyorkartistpt410"
arm_ip = "144.32.152.2"
token = "4b1c26278a566e0419165a3acd025dd83d32b160"
# arm_hostname = "trendylimashifterpt410"
# arm_ip = "144.32.152.105"
# token = "1462980d67d58cb7eaabe8790d866609eb97fd2c"

eva = Eva(arm_ip, token)

def dopen():
    with eva.lock():
       eva.control_wait_for_ready()
       eva.gpio_set('ee_d0', False)
       eva.gpio_set('ee_d1', True)

def dclose():
    with eva.lock():
       eva.control_wait_for_ready()
       eva.gpio_set('ee_d0', True)
       eva.gpio_set('ee_d1', False)

def radius2Degree(degree):
    return degree * math.pi / 180

r = radius2Degree

initThetaM = [r(-20), r(-83.5), -(r(90)-r(40)), r(70), -r(100), r(45)]

f = pickle.load(open('parameter', 'rb'))  
ret, mtx, dist, rvecs, tvecs = f['ret'], f['mtx'], f['dist'], f['rvecs'], f['tvecs']

def resetPos():
    with eva.lock():
        eva.control_wait_for_ready()
        eva.control_go_to(initThetaM)

class MyThread(threading.Thread):
    def __init__(self, threadID, name, whichHandler):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.handler = whichHandler

    def run(self):
        self.asyncEva()

    @classmethod
    def asyncEva(self):
        global r_distance
        time.sleep(3)
        step = 0.01
        thetaM = initThetaM
        lastThetaM = initThetaM
        lastDiff = None

        dopen()
        lastMotion = 'left'
        while True:
            diff = obj_x - C_x
            if lastDiff != None and (diff > 0 and lastDiff < 0) or (diff < 0 and lastDiff > 0):
                print('===> overflow')
                break
            lastDiff = diff
            if step == 0:
                continue
            if abs(diff) >= 150:
                if diff > 0:
                    lastThetaM= goarm(0, step, -0, thetaM)
                    step += 0.007
                    lastMotion = 'right'
                else:
                    lastThetaM = goarm(0, step, -0, thetaM)
                    step -= 0.007
                    lastMotion = 'left'

                print("-> exceed ", diff)
            else:
                break
            time.sleep(0.5)

        print('move cm ----> ', r_distance)
        if r_distance > 6 or r_distance <= 1.5:
            print('reset distance!')
            r_distance = 3
        if lastMotion == 'left':
            goarm(r_distance/100, 0, -0, lastThetaM)
        else:
            goarm(r_distance/100, 0, -0, lastThetaM)
        dclose()
        time.sleep(2)
        with eva.lock():
            eva.control_wait_for_ready()
            eva.control_go_to([r(-55), r(-20), -(r(90)+r(0)), 0, -(r(90)-r(20)), r(0)])
        dopen()
        send(log)
        return


def goarm(_x, _y, _z,
    init_cord=initThetaM
):
    xyz = [_x, _y, _z]
    print("-> x,y,z--->", xyz)

    theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')
    theta = Matrix([theta1, theta2, theta3, theta4, theta5, theta6])
    step_size = 0.0005
    theta_max_step = 0.4

    def T(x, y, z):
        T_xyz = Matrix([[1, 0, 0, x],
                        [0, 1, 0, y],
                        [0, 0, 1, z],
                        [0, 0, 0, 1]])
        return T_xyz


    def Rx(roll):
        R_x = Matrix([[1, 0, 0, 0],
                      [0, cos(roll), -sin(roll), 0],
                      [0, sin(roll), cos(roll), 0],
                      [0, 0, 0, 1]])
        return R_x


    def Ry(pitch):
        R_y = Matrix([[cos(pitch), 0, sin(pitch), 0],
                      [0, 1, 0, 0],
                      [-sin(pitch), 0, cos(pitch), 0],
                      [0, 0, 0, 1]])
        return R_y


    def Rz(yaw):
        R_z = Matrix([[cos(yaw), -sin(yaw), 0, 0],
                      [sin(yaw), cos(yaw), 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        return R_z


    def R(roll, pitch, yaw):
        R_x = Matrix([[1, 0, 0, 0],
                      [0, cos(roll), -sin(roll), 0],
                      [0, sin(roll), cos(roll), 0],
                      [0, 0, 0, 1]])
        R_y = Matrix([[cos(pitch), 0, sin(pitch), 0],
                      [0, 1, 0, 0],
                      [-sin(pitch), 0, cos(pitch), 0],
                      [0, 0, 0, 1]])
        R_z = Matrix([[cos(yaw), -sin(yaw), 0, 0],
                      [sin(yaw), cos(yaw), 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        return R_z * R_y * R_x


    # Define transforms to each joint
    T1 = Ry(-pi / 2) * T(0.187, 0, 0) * Rx(theta1)
    T2 = T1 * T(0.096, 0, 0) * Rz(theta2)
    T3 = T2 * T(0.205, 0, 0) * Rz(theta3)
    T4 = T3 * T(0.124, 0, 0) * Rx(theta4)
    T5 = T4 * T(0.167, 0, 0) * Rz(theta5)
    T6 = T5 * T(0.104, 0, 0)  # fixed point

    # Find joint positions in space
    p0 = Matrix([0, 0, 0, 1])
    p1 = T1 * p0
    p2 = T2 * p0
    p3 = T3 * p0
    p4 = T4 * p0
    p5 = T5 * p0
    p6 = T6 * p0
    p = Matrix([p6[0], p6[1], p6[2]])  # coordinates of arm tip
    J = p.jacobian(theta)
    #   eva.control_go_to([0, 0, -(r(90)+r(30)), 0, -(r(90)-r(30)), r(10)])

    theta_i = Matrix(
        init_cord
    )  # initial position θi #theta_i = Matrix([0,0,0,0,0,0]
    p_i = p.subs({theta1: theta_i[0], theta2: theta_i[1], theta3: theta_i[2], theta4: theta_i[3], theta5: theta_i[4],
                  theta6: theta_i[5]}).evalf()
    # cm
    p_f = p_i + Matrix(
        xyz
    )  # moving the arm down in the z axis by 1cm
    dp = p_f - p_i  # distance between end effector is now and our goal position
    dp_threshold = 0.001
    path = []
    while dp.norm() > dp_threshold:
        dp_step = dp * step_size / dp.norm()
        J_i = J.subs({theta1: theta_i[0], theta2: theta_i[1], theta3: theta_i[2], theta4: theta_i[3], theta5: theta_i[4],
                      theta6: theta_i[5]}).evalf()
        J_inv = J_i.pinv()
        dtheta = J_inv * dp_step
        theta_i = theta_i + numpy.clip(dtheta, -1 * theta_max_step, theta_max_step)
        p_i = p.subs({theta1: theta_i[0], theta2: theta_i[1], theta3: theta_i[2], theta4: theta_i[3], theta5: theta_i[4],
                      theta6: theta_i[5]}).evalf()
        dp = p_f - p_i
        path.append(theta_i)

    path = [list(i) for i in path]

    path = path[::10]

    path1 = []
    path2 = [{
        "type": "home",
        "waypoint_id": 0
    }]
    for count, val in enumerate(path):
        val2 = [float(i) for i in val]
        idx = count + 1
        path1.append({
            "label_id": idx,
            "joints": val2
        })
        path2.append({
            "type": "trajectory",
            "trajectory": "joint_space",
            "waypoint_id": idx
        })

    step = len(path1)
    next_step = step + 1
    toolpath = {
        "metadata": {
            "version": 2,
            "default_max_speed": 1,
            "payload": 0,
            "analog_modes": {
                "i0": "voltage",
                "i1": "voltage",
                "o0": "voltage",
                "o1": "voltage"
            },
            "next_label_id": next_step
        },
        "waypoints": path1[:step]
        ,
        "timeline": path2[:step]
    }

    thetaM = [i[0] for i in theta_i.tolist()]
    log['theta'] = [float(i[0]) for i in theta_i.tolist()]

    eva = Eva(arm_ip, token)

    with eva.lock():
        eva.control_wait_for_ready()
        eva.toolpaths_use(toolpath)
        eva.control_home()
        eva.control_run(loop=1, mode='teach')

    return thetaM


resetPos()
time.sleep(2)


camera_hostname = "evacctv02"
camera_ip = "144.32.152.11"
cam = Camera("S1188413")

# camera_hostname = "evacctv03"
# camera_ip = "144.32.152.10"
# cam = Camera("S1188411")

cam.set_feature("Width", 1936)
cam.set_feature("Height", 1216)
cam.set_frame_rate(10)
cam.set_exposure_time(1000000)
cam.set_pixel_format_from_string('BayerRG8')

print("Camera model: ", cam.get_model_name())
print("Vendor Name: ", cam.get_vendor_name())
print("Device id: ", cam.get_device_id())
print("Region: ", cam.get_region())

if rArm:
    thread1 = MyThread(1, 'eva',  'eva')
    thread1.start()
try:
   cam.start_acquisition_continuous()
   print("Camera On")
   cv2.namedWindow('go', flags=0)
   count = 0
   sp = Shape()
   while True:
       count += 1
       frame = cam.pop_frame()
       if not 0 in frame.shape:
          img = cv2.cvtColor(frame, cv2.COLOR_BayerRG2RGB)
          hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
          red1, red2 = (100,100, 50), (179, 255, 255)
          green1, green2 =(50, 80, 80), (80, 255, 255) 
          mask = cv2.inRange(hsv, green1, green2)

          contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
          threshold = 500

          try:
              cs = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
              cs = imutils.grab_contours(cs)
              c2 = max(cs, key=cv2.contourArea)
              M2 = cv2.moments(c2)
              mark, centre = cv2.minAreaRect(c2), [int(M2["m10"] / M2["m00"]), int(M2["m01"] / M2["m00"])]
              def getFocal(mark, W, D):
                  P = mark[1][0] / pxl 
                  return (P * D) / W  
              def my_distance_to_camera(F, P, W):
                  return (F * W) / P 
              W = 3
              F = 28.585333218232048
              P = mark[1][0] / pxl 
              inches = my_distance_to_camera(F, P, W) #
              griper_length = 19
              r_distance = inches - griper_length
          except Exception as ex:
              print(ex)

          for c in contours:
               perimeter = cv2.arcLength(c, True)
               if perimeter < threshold:
                   continue
               log['perimeter'] = perimeter
               img = cv2.drawContours(img, [c], -1, (0,255,255), 3)

               M = cv2.moments(c)
               try:
                   obj_x = int(M['m10']/M['m00'])
                   obj_y = int(M['m01']/M['m00'])
                   cv2.circle(img,(obj_x,obj_y),7,128,-1)
                   str1 = '(' + str(obj_x)+ ',' +str(obj_y) +')'
                   cv2.putText(img,str1,(obj_x-50,obj_y+40),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)#绘制坐标点位

                   deltaX = (C_x - obj_x) / pxl / 100
                   deltaY = (obj_y - C_y) / pxl / 100

               except Exception as ex:
                   pass

       cv2.imshow("go", img)
       cv2.waitKey(1)
except KeyboardInterrupt:
   print("Exiting...")
finally:
   cam.stop_acquisition()
   cam.shutdown()
   print("Camera Off")
