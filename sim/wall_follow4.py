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
angle_values = []
avr_num = 5
pre = None
scan = None

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
    global angle_sum, avr, scan, pre,cnt

    angle = 0.0
    speed = 0.0
    LEFT = -1
    RIGHT = 1
    mode = 0
    
    # Use the triggers to control the car's speed
    """
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt

    if rc.controller.is_down(rc.controller.Button.X):
        speed = 0.5
    """
    pre_scan = rc.lidar.get_samples()
    if cnt % 60 == 0:
        scan = pre_scan
    cnt += 1
    #scan = lidar.get_lidar()
    #print(scan)
    

    left_front_dist = rc_utils.get_lidar_average_distance(scan, -52, 38)
    right_front_dist = rc_utils.get_lidar_average_distance(scan, 52, 38)
    left_rear_dist = rc_utils.get_lidar_average_distance(scan, -128, 38)
    right_rear_dist = rc_utils.get_lidar_average_distance(scan, -128, 38)

    r_dis = scan[180]
    fr_dis = scan[0]


    if rc.controller.is_down(rc.controller.Button.A):
        
        kp = 0.025
        num = 0
        """
        _, right_wall_dist = rc_utils.get_lidar_closest_point(scan, (10, 75))
        _, left_wall_dist = rc_utils.get_lidar_closest_point(scan, (-75, -10))

        left_front_dist = rc_utils.get_lidar_average_distance(scan, -38, 76)
        right_front_dist = rc_utils.get_lidar_average_distance(scan, 38, 76)
        speed = 0.3
        if right_wall_dist < 30:
            angle = LEFT
            mode = 0
        elif left_wall_dist < 30:
            angle = RIGHT
            mode = 1
        elif right_front_dist >70:
            angle = RIGHT
            mode = 2
        elif left_front_dist > 70:
            angle = LEFT
            mode = 3
        else:
            speed = 0.2
            angle = 0
            mode = 4
        """
        angle = kp * (right_front_dist - right_rear_dist - left_front_dist + left_rear_dist)
        if right_front_dist == 0 or right_rear_dist == 0:
            angle = 2 * kp * (- left_front_dist + left_rear_dist)
        if left_front_dist == 0 or left_rear_dist == 0:
            angle = 2 * kp * (right_front_dist - right_rear_dist)
        speed = 0.5
        
    _, forward_wall_dist = rc_utils.get_lidar_closest_point(scan, (-10, 10))
    
    if forward_wall_dist < 100:
        """
        if right_front_dist - right_rear_dist - left_front_dist + left_rear_dist >= 0:
            angle = LEFT
        else:
            angle = RIGHT
        """

        speed = 1
        angle = LEFT
        mode = 5
        
    angle_values.insert(0, angle)

    if len(angle_values) <= avr_num:
        avr = average(angle_values)
    elif len(angle_values) > avr_num:
        _ = angle_values.pop()
        avr = average(angle_values)
    #print(angle_values)
    #print(f"mode {mode}")
    """
    r_list = scan[90:270]
    r_min = min(r_list)

    k = 0.1
    goal_dis = 35
    limit_dis = 200
    turn_dis = 100
     
    if forward_wall_dist < limit_dis:
        goal_dis = 15

    avr = k * (r_min - goal_dis)

    if forward_wall_dist < turn_dis:
        avr = -1
        speed = 1
    """

    avr = np.clip(avr, -1, 1)

    rc.drive.set_speed_angle(speed, avr)
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