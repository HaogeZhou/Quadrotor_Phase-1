import numpy as np
from math import cos, sin

# Given time and coeffs of quintic fuction to calcute position
def calculate_pos(c, t):

    pos = c[0] * t ** 5 + c[1] * t ** 4 + c[2] * t ** 3 + c[3] * t ** 2 + c[4] * t + c[5]
    return pos


# Given time and  coeffs of quintic fuction to calcute velocity
def calculate_vel(c, t):

    vel= 5 * c[0] * t ** 4 + 4 * c[1] * t ** 3 + 3 * c[2] * t ** 2 + 2 * c[3] * t + c[4]

    return vel
# Given time and coeffs of quintic fuction to calcute acceleration
def calculate_acc(c, t):

    acc = 20 * c[0] * t ** 3 + 12 * c[1] * t ** 2 + 6 * c[2] * t + 2 * c[3]
   
    return acc

# Calculate rotation matrix
def rotation_matrix(roll, pitch, yaw):

    rot_m=np.array(
        [[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll),
          sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll)],
         [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) *
          sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll)],
         [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(yaw)]
         ])

    return rot_m