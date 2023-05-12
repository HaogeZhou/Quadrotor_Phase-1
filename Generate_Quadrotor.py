import numpy as np
from math import cos, sin
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import imageio


class Quadrotor():
    # draw Quadrotor
    def plot(self):
        fig = plt.figure()
        ax = Axes3D(fig)
        T = self.transformation_matrix()

        # Draw 4 black dot
        p1_t = np.matmul(T, self.p1)
        p2_t = np.matmul(T, self.p2)
        p3_t = np.matmul(T, self.p3)
        p4_t = np.matmul(T, self.p4)

        plt.cla()

        # Draw Quadrotor 
        ax.plot([p1_t[0], p2_t[0], p3_t[0], p4_t[0]],
                     [p1_t[1], p2_t[1], p3_t[1], p4_t[1]],
                     [p1_t[2], p2_t[2], p3_t[2], p4_t[2]], 'k.')

        ax.plot([p1_t[0], p2_t[0]], [p1_t[1], p2_t[1]],
                     [p1_t[2], p2_t[2]], 'r-')
        ax.plot([p3_t[0], p4_t[0]], [p3_t[1], p4_t[1]],
                     [p3_t[2], p4_t[2]], 'r-')

        ax.plot(self.x_data, self.y_data, self.z_data, 'b:')

        plt.xlim(-20, 20)
        plt.ylim(-20, 20)
        ax.set_zlim(0, 20)
        plt.pause(0.01)
        plt.savefig('trajectory.png')  # save a frame
        plt.show()
        plt.close()
        # self.image_list.append(imageio.imread('trajectory.png'))  # read a frame of image and save in list for generate gif
        # imageio.mimsave('pic.gif', self.image_list, duration=100)   # save gif

    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, size=100, show_animation=True):
        self.p1 = np.array([size / 2, 0, 0, 1]).T
        self.p2 = np.array([-size / 2, 0, 0, 1]).T
        self.p3 = np.array([0, size / 2, 0, 1]).T
        self.p4 = np.array([0, -size / 2, 0, 1]).T

        self.x_data = []
        self.y_data = []
        self.z_data = []
        self.show_animation = show_animation

        plt.ion()

        fig = plt.figure()
        self.ax = fig.add_subplot(111, projection='3d')
        self.image_list = []  # list of image for draw gif

        self.update_pose(x, y, z, roll, pitch, yaw)

    # renew position args
    def update_pose(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)

        if self.show_animation:
            self.plot() 

    # renew matrix
    def transformation_matrix(self):
        x = self.x
        y = self.y
        z = self.z
        roll = self.roll
        pitch = self.pitch
        yaw = self.yaw
        trans_m = np.array(
            [[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll), x],
             [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) *
              sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll), y],
             [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(yaw), z]
             ])

        return trans_m


   