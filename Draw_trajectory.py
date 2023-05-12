from math import cos, sin
import numpy as np
import matplotlib.pyplot as plt
from Generate_Quadrotor import Quadrotor
from Generate_Trajectory import TrajectoryGenerator
from Function import rotation_matrix, calculate_acc,calculate_pos,calculate_vel

show_animation = True

# Simulation parameters
g = 9.81
m = 0.03
Ixx = 1.43e-5
Iyy = 1.43e-5
Izz = 2.89e-5
T = 5

# Proportional coefficients
Kp_x = 2e-5
Kp_y = 2e-5
Kp_z = 2e-5
Kp_roll = 2e-4
Kp_pitch = 2e-4
Kp_yaw = 2e-4

# Derivative coefficients
Kd_x = 0.1
Kd_y = 0.1
Kd_z = 0.1

# Calculates the necessary thrust and torques for the quadrotor
def quad_sim(hover_or_not, hover_time, start_pos, x_c, y_c, z_c):
    """
    follow the trajectory described by the sets of coefficients
    x_c, y_c, and z_c.
    """
    x_pos = start_pos[0]  # initinal position 
    y_pos = start_pos[1]
    z_pos = start_pos[2]
    x_vel = 0
    y_vel = 0
    z_vel = 0
    x_acc = 0
    y_acc = 0
    z_acc = 0
    roll = 0 # Angular position about the x-axis in radians.
    pitch = 0 # Angular position about the y-axis in radians.
    yaw = 0 # Angular position about the z-axis in radians.
    roll_vel = 0
    pitch_vel = 0
    yaw_vel = 0
    des_pitch=0
    des_roll=0

    des_yaw = 1

    dt = 0.2
    t = 0
    t_hover = 0  # 悬停初始时间
    all_flight_time = 0  # 整个飞行时间记录

    q = Quadrotor(x=x_pos, y=y_pos, z=z_pos, roll=roll,
                  pitch=pitch, yaw=yaw, size=4, show_animation=show_animation)

    i = 0
    n_run = 3
    irun = 0

    while True:
        if hover_or_not[i]:  # 悬停判断
            while t_hover <= hover_time[i]:  # 悬停时间
                q.update_pose(x_pos[0], y_pos[0], z_pos[0], roll[0], pitch[0], yaw)
                t_hover += dt
        all_flight_time += t_hover  # 添加悬停时间
        while t <= T:
            des_x_pos = calculate_pos(x_c[i], t)
            des_y_pos = calculate_pos(y_c[i], t)
            des_z_pos = calculate_pos(z_c[i], t)
            des_x_vel = calculate_vel(x_c[i], t)
            des_y_vel = calculate_vel(y_c[i], t)
            des_z_vel = calculate_vel(z_c[i], t)
            # des_pitch_vel = calculate_vel(pitch_c[i], t)
            des_x_acc = calculate_acc(x_c[i], t)
            des_y_acc = calculate_acc(y_c[i], t)
            des_z_acc = calculate_acc(z_c[i], t)

            thrust = m * (g + des_z_acc + Kp_z * (des_z_pos -z_pos)+ Kd_z * (des_z_vel - z_vel))

            # moment = Ixx(des_pitch_acc + kd_pitch(des_pitch_vel- pitch_vel)+kp_pitch(des_pitch-pitch))

            roll_torque = Kp_roll * (((des_x_acc * sin(des_yaw) - des_y_acc * cos(des_yaw)) / g) - roll)
            pitch_torque = Kp_pitch * (((des_x_acc * cos(des_yaw) - des_y_acc * sin(des_yaw)) / g) - pitch)
            yaw_torque = Kp_yaw * (des_yaw - yaw)

            roll_vel += roll_torque * dt / Ixx
            pitch_vel += pitch_torque * dt / Iyy
            yaw_vel += yaw_torque * dt / Izz

            roll += roll_vel * dt
            pitch += pitch_vel * dt
            yaw += yaw_vel * dt

            R = rotation_matrix(roll, pitch, yaw)
            acc = (np.matmul(R, np.array([0, 0, thrust]).T) - np.array([0, 0, m * g]).T) /m
            x_acc = acc[0]
            y_acc = acc[1]
            z_acc = acc[2]
            x_vel += x_acc * dt
            y_vel += y_acc * dt
            z_vel += z_acc * dt
            x_pos += x_vel * dt
            y_pos += y_vel * dt
            z_pos += z_vel * dt

            q.update_pose(x_pos[0], y_pos[0], z_pos[0], roll[0], pitch[0], yaw)  # 更新位姿参数

            t += dt
        all_flight_time += t
        t = 0
        t_hover = 0
        i = (i + 1) % 4
        irun += 1
        if irun >= n_run:
            break
    print('The entire flight time is ', all_flight_time, 's')
    print("Done")


def main():
    """
    Calculates the x, y, z coefficients for the four segments 
    of the trajectory
    """
    x_coeffs = [[], [], [], []]
    y_coeffs = [[], [], [], []]
    z_coeffs = [[], [], [], []]
    waypoints = [[0, 0, 0], [5, 5, 5], [5, 10, 5], [5, 10, 0]]
    # waypoints = [[10,  5,0], [10, 5, 5], [-5, 5, 5], [-5, 5, 0]]
    # waypoints = [[15, 20, 0], [15, 20, 5], [-5, -15, 5], [-5, -15, 0]]  # 路径关键点
    hover_or_not = [False, False, True]  # 每一段初始位置是否悬停标志位
    hover_time = [0, 0, 5]  # 每一段初始位置是否悬停时间

    for i in range(4):
        traj = TrajectoryGenerator(waypoints[i], waypoints[(i + 1) % 4], T)  # initialize
        traj.solve()  # solve each line quintic polynomial trajectory.
        x_coeffs[i] = traj.x_c
        y_coeffs[i] = traj.y_c
        z_coeffs[i] = traj.z_c

    quad_sim(hover_or_not, hover_time, waypoints[0], x_coeffs, y_coeffs, z_coeffs)



if __name__ == "__main__":
    main()
