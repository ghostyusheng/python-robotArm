from evasdk import Eva
import math




def radiusToDegree(degree):
    return degree * math.pi / 180

r = radiusToDegree

def move(joint1Degree=0,joint2Degree=0,joint3Degree=-90,joint4Degree=0,joint5Degree=-90,joint6Degree=0):
    joint1Degree = r(joint1Degree)
    joint2Degree = r(joint2Degree)
    joint3Degree = r(joint3Degree)
    joint4Degree = r(joint4Degree)
    joint5Degree = r(joint5Degree)
    joint6Degree = r(joint6Degree)
    arm_hostname = "tevacunningyorkartistpt410"
    arm_ip = "144.32.152.2"
    token = "4b1c26278a566e0419165a3acd025dd83d32b160"
    eva = Eva(arm_ip, token)
    with eva.lock():
        eva.control_wait_for_ready()
        #eva.control_go_to([0, 0, -(r(90)+r(30)), 0, -(r(90)-r(30)), r(10)])
        eva.control_reset_errors()
        #eva.control_go_to([joint1Degree, joint2Degree, joint3Degree, joint4Degree, joint5Degree, joint6Degree]) 
move(20)
