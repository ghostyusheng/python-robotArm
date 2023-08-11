import numpy
import matplotlib.pyplot
import sympy
from mpl_toolkits.mplot3d import Axes3D
from sympy import Matrix, Symbol, symbols, solveset, solve, simplify, S, diff, det, erf, log, sqrt, pi, sin, cos, tan, asin, acos, atan2, init_printing, pretty_print

'''
p0 = Matrix([0, 0, 0, 1])
p1 = Matrix([0, 0, 0.187, 1])
p2 = Matrix([0, 0, 0.187+0.096, 1])
p3 = Matrix([0.205*cos(pi/4), 0, 0.187+0.096+0.205*sin(pi/4), 1])
p4 = Matrix([0.205*cos(pi/4)+0.124, 0, 0.187+0.096+0.205*sin(pi/4), 1])
p5 = Matrix([0.205*cos(pi/4)+0.124+0.167, 0, 0.187+0.096+0.205*sin(pi/4), 1])
p6 = Matrix([0.205*cos(pi/4)+0.124+0.167, 0,
            0.187+0.096+0.205*sin(pi/4)-0.104, 1])
'''

'''theta1=-pi/2
theta2=-pi/4
theta3=3*pi/4
theta4=0
theta5=pi/2
theta6=0'''

theta1,theta2,theta3,theta4,theta5,theta6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')
theta = Matrix([theta1,theta2,theta3,theta4,theta5,theta6])

step_size = 0.0005
theta_max_step = 0.2

# Not necessary but gives nice-looking latex output
# More info at: http://docs.sympy.org/latest/tutorial/printing.html
# init_printing()


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
'''def R(roll, pitch, yaw):
    R_x = Matrix([[1, 0, 0],
              [0, cos(roll), -sin(roll)],
              [0, sin(roll), cos(roll)]])
    R_y = Matrix([[cos(pitch), 0, sin(pitch)],
              [0, 1, 0],
              [-sin(pitch), 0, cos(pitch)]])
    R_z = Matrix([[cos(yaw), -sin(yaw), 0],
              [sin(yaw), cos(yaw), 0],
              [0, 0, 1]])
    return R_z*R_y*R_x'''


def R(roll, pitch, yaw):
  R_x = Matrix([[1, 0, 0, 0],
              [0, cos(roll), -sin(roll), 0],
              [0, sin(roll),  cos(roll), 0],
              [0, 0, 0, 1]])
  R_y = Matrix([[ cos(pitch), 0, sin(pitch), 0],
              [0, 1, 0, 0],
              [-sin(pitch), 0, cos(pitch), 0],
              [0, 0, 0, 1]])
  R_z = Matrix([[cos(yaw),-sin(yaw), 0, 0],
              [sin(yaw), cos(yaw), 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])
  return R_z*R_y*R_x

# Define transforms to each joint
T1 = Ry(-pi/2) * T(0.187, 0, 0) * Rx(theta1)
T2 = T1 * T(0.096, 0, 0) * Rz(theta2)
T3 = T2 * T(0.205, 0, 0) * Rz(theta3)
T4 = T3 * T(0.124, 0, 0) * Rx(theta4)
T5 = T4 * T(0.167, 0, 0) * Rz(theta5)
T6 = T5 * T(0.104, 0, 0) # fixed point

# Find joint positions in space
p0 = Matrix([0,0,0,1])
p1 = T1 * p0
p2 = T2 * p0
p3 = T3 * p0
p4 = T4 * p0
p5 = T5 * p0
p6 = T6 * p0
# print('T1=',T1,'\n\nT2=',T2,'\n\nT3=',T3,'\n\nT4=',T4,'\n\nT5=',T5,'\n\nT6=',T6)
# print('p1=',p1,'\n\np2=',p2,'\n\np3=',p3,'\n\np4=',p4,'\n\np5=',p5,'\n\np6=',p6)
p = Matrix([p6[0], p6[1], p6[2]]) # coordinates of arm tip
J = p.jacobian(theta)
theta_i = Matrix([-pi/2,-pi/4,3*pi/4,0,pi/2,0]) # initial position θi #theta_i = Matrix([0,0,0,0,0,0]
p_i = p.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
p_f = p_i + Matrix([0.00, 0.0, 0.01]) # moving the arm down in the z axis by 1cm
dp = p_f - p_i  #  distance between end effector is now and our goal position

print("p_i:", p_i)
dp_threshold = 0.001

while dp.norm() > dp_threshold:
    
    dp_step = dp * step_size / dp.norm()
    print("dp_step: ",dp_step)
    J_i = J.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
    J_inv = J_i.pinv()
    dtheta = J_inv * dp_step
    theta_i = theta_i + numpy.clip(dtheta,-1*theta_max_step,theta_max_step)
    p_i = p.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
    dp = p_f - p_i
    
    print("step “,step,”:\n θ: [",theta_i,"]\n p:[",p_i,"]")