To run the code, just directedly run the main.py. 
The results are already in these png and gif files.
The default condition is from(0, 0, 0) to (0,0,10) to (10,0,10) and hover for 5 seconds, then landing at(10,0,0).

The main.py includes some basic calculation and elements which we need for the further operation.
Thrust's formula is used based on the lecture note. Also, in this file, includes pitch torque, roll torque,and yaw torque based on proportional controller (P controller). For the roll torque, it is equal to Kp_roll * (roll desired - roll current), and roll desired is equal to  (des_x_acc * sin(des_yaw) - des_y_acc * cos(des_yaw)) / g, and roll current is current value. In this case, derivative term (D controller) is not been used. If try to include derivative term (D controller), the whole term will be different which is roll_torque = Kp_roll * (roll_desired - roll) + Kd_roll * (roll_rate_desired - roll_rate_current), and the code should define the variables Kd_roll, roll_rate_desired, and roll_rate_current, and calculate the desired roll rate based on the control strategy. 
The planar control equations can be found on lecture note 06, page 27. Advanced Robotics; Instructor: K. Karydis.

The Generate_Quadrotor.py is about build up the Quadrotor, including define initinal position and update position, how to calcute transition matrix and plot quadrotor in 3D.
For Generate_Trajectory.py, because we have to deal with six different elements which includ x, y, z, pitch, roll, and yaw. Based on that we have to use quintic polynomial equation, for using that
it can help us to get better trajectory with continuous position, velocity, and accelertation. For the quintic polynomial equation, since we have 6 elements, so it provides enough degree of freedom to satisfy the demand.
And based on that we can have a 6*6 matrix which includes initial position, final position, initial velocity, final velocity, initial acceleration, and final acceleration. If degree 4 polynomial equation been used, it will
not satisfy our demand which cannot meet all constraints. If try to use high order polunomial equations such as degree of 7, the result can be more complex which is not what we want. So in this case, only quintic polynomial equation can balance pros and cons.

The Function.py is the definition of calcute velocity, acceleration and rotation matrix as what is mentioned previously. In order to get more smooth trajectory quintic polynomial equation is needed. The initial quintic polynomial equation is for the initial postion, once we take the first derivative of that we can get the velocity, and second derivative is for acceleration. For the rotation matrix, since we have the pitch, roll, and yaw, it will be a 3*3 matrix based on the formula.

The trajectory.png is the trajectory of Quadrotor.
The pic.gif is the gif file about the Quadrotor. You can unmute the line 42 43 in Generate_Quadrotor.py to save the gif. There is an example gif file in Example.gif.  
The Velocity and Acceleration are shown in their png file.

Reference:
Lecture note
https://github.com/udacity/RoboND-Controls-Lab
https://github.com/MountainXiu/PythonRobotics
https://github.com/uzh-rpg/rpg_quadrotor_control
