"""
MIT BWSI Autonomous RACECAR
MIT License
bwsix RC101 - Fall 2023

File Name: lab_g.py

Title: Lab G - Line Follower with Safety Stop

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: Write a script to enable fully autonomous behavior from the RACECAR. The
RACECAR should automatically identify the color of a line it sees, then drive on the
center of the line throughout the obstacle course. The RACECAR should also identify
color changes, following colors with higher priority than others. When the RACECAR sees
a white cone at the end of the course, it must stop automatically before hitting the cone.

Note: You may copy code from Lab F to complete Lab G. You are expected to build on top
of your progress and add state machine logic and safety stop features to your code.
There is no template code in this document to follow except for the RACECAR script structure
found in template.py. The Grand Prix will be very similar to this lab. If you have time,
try optimizing your line following algorithm. Good luck!

Expected Outcome: When the user runs the script, they must not be able to manually control
the RACECAR. The RACECAR must move forward on its own, traverse through the course, and then
stop on its own.
- The speed of the RACECAR can be controlled by a state machine or script, but not by the user
- The angle of the RACECAR should only be controlled by the center of the line contour
- The RACECAR sees the color RED as the highest priority, then GREEN, then BLUE
- The RACECAR must stop before the white cone at the end of the course. The RACECAR must stop
close enough to the cone such that it does not see the entirety of the white cone at the end
of the race. (less than 30 cm). The RACECAR must not hit the white cone.
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils
from real_lidar import real_lidar

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
#FRONT_WINDOW = (-10, 10)
#REAR_WINDOW = (170, 190)

class PID:
    def __init__(self, Kp, Ki, Kd ,dt=1/60):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0
        self.dt = dt
    
    def update(self, error):
        dt = self.dt
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output
    
a1 = 0.05
a2 = 0.2
d =0.1
    
kp_a1 = 1 * a1
ki_a1 = 0.05 * a1
kd_a1 = 0.05 * a1
kp_a2 = 1 * a2
ki_a2 = 0.05 * a2
kd_a2 = 0.05 * a2
kp_d = 1 * d
ki_d = 0.05 * d
kd_d = 0.05 * d


pid_angle_1 = PID(kp_a1,ki_a1,kd_a1)
pid_angle_2 = PID(kp_a1,ki_a1,kd_a1)
pid_dist = PID(kp_d,ki_d,kd_d)
goal_dis = 35
limit_dist = 100
angle = 0
speed  =0


lidar = real_lidar(n=20)

cnt = 0
########################################################################################
# Functions
########################################################################################

def average(angle_values):
    
    angle = 0
    ratio = 0.6
    #print(angle_values)
    angles = angle_values.copy()
    angles.reverse()
    for i in angles:
        angle += i
        angle *= ratio
    
    #angle = sum(angle_values)/len(angle_values)
    angle = np.clip(angle, -1, 1)
    return angle


# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    # Remove 'pass' and write your source code for the start() function here
    rc.drive.stop()


    # Print start message
    print(">> Lab 4B - LIDAR Wall Following")


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global pid_angle_1, pid_angle_2, pid_dist, goal_dis, limit_dist, angle, speed

    scan = rc.lidar.get_samples()

    left_front_dist = rc_utils.get_lidar_average_distance(scan, -52, 38)
    right_front_dist = rc_utils.get_lidar_average_distance(scan, 52, 38)
    left_rear_dist = rc_utils.get_lidar_average_distance(scan, -128, 38)
    right_rear_dist = rc_utils.get_lidar_average_distance(scan, -128, 38)

    _, forward_wall_dist = rc_utils.get_lidar_closest_point(scan, (-10, 10))

    r_dis = scan[180]

    r_list = scan[90:270]
    r_min = min(r_list)

    error_a1 = np.arccos(r_min / r_dis)
    if r_list[30] < r_list[-30]:
        error_a1 *= -1

    error_a2 = right_front_dist - right_rear_dist

    error_d = r_min - goal_dis

    angle_1 = pid_angle_1.update(error_a1)
    angle_2 = pid_angle_2.update(error_a2)
    dist = pid_dist.update(error_d)

    angle = np.clip(angle_1 + angle_2 + dist, -1, 1)

    speed = 0
    if rc.controller.is_down(rc.controller.Button.A):
        speed = 0.5

#    if forward_wall_dist < limit_dist:
#        angle = -1
        
    print(f"angle|{angle}")
    print(f"dist|{error_d}")

    rc.drive.set_speed_angle(speed, angle)
    #print(avr)
    #print(r_min)
    #print(left_front_dist)

    # Print the current speed and angle when the A button is held down
    

    # Print the center and area of the largest contour when B is held down
    # Remove 'pass' and write your source code for the update() function here

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()

    plt.plot(angle_values)
    plt.xlabel('Time')
    plt.ylabel('Angle')
    plt.title('Angle over Time')
    plt.show()