"""
MIT BWSI Autonomous RACECAR
MIT License
bwsix RC101 - Fall 2023

File Name: template.py << [Modify with your own file name!]

Title: [PLACEHOLDER] << [Modify with your own title]

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: [PLACEHOLDER] << [Write the purpose of the script here]

Expected Outcome: [PLACEHOLDER] << [Write what you expect will happen when you run
the script.]
"""

########################################################################################
# Imports
########################################################################################

import sys
import numpy as np

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

mode = None
k1 = 0.2
k2 = 0.0
kd = 0.1

# Declare any global variables here


########################################################################################
# Functions
########################################################################################

def sigmoid(x):
    a = 1 + np.exp(-x)
    return 1/a

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    pass # Remove 'pass' and write your source code for the start() function here

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global mode, k1, k2, kd

    limit_dis = 200
    turn_dis = 100
    goal_dis = 100

    scan = rc.lidar.get_samples()
    
    
    fr_dis = scan[0]
    frr_dis = scan[90]
    r_dis = scan[180]
    fwr_dis = scan[270]
    fw_dist  =scan[360]
    fwl_dis = scan[450]
    l_dis = scan[540]
    frl_dist = scan[630]
    
    r_list = scan[90:270]
    r_min = min(r_list)

    if fwr_dis == None:
        pass
    
    frr_angle = np.arccos(r_min / frr_dis)
    fwr_angle = np.arccos(r_min / fwr_dis)
    
    right_front_dist = rc_utils.get_lidar_average_distance(scan, 52, 38)
    right_rear_dist = rc_utils.get_lidar_average_distance(scan, -128, 38)

    #angle_1 = (fwr_angle - frr_angle) / (2 * np.pi)
    angle_2 = right_front_dist - right_rear_dist

    angle_1 = np.arccos(r_min / r_dis)
    #if frr_dis < fwr_dis:
    if r_list[120] > r_list[-120]:
        angle_1 *= -1
    
    """
    sin = 1/np.sin(np.pi/2)
    
    if fr_dis >= limit_dis:
        goal_dis = 50
    

        #angle =-k * ((frr_dis + r_dis + fwr_dis) / 3 - goal_dis)
        angle = k * (right_front_dist - right_rear_dist)

    if fr_dis < limit_dis:
        goal_dis = 50
        if fr_dis >= turn_dis:
            angle = k * (r_dis - goal_dis)

        else:
            angle = 1
            """

    if rc.controller.is_down(rc.controller.Button.A):
       goal_dis = 60
    #angle = (right_front_dist - right_rear_dist) #+ kd *(r_dis - goal_dis)
    #angle = k * ((sin * frr_dis + r_dis + sin *  fwr_dis) / 3 - goal_dis)
    #print((frr_dis + r_dis + fwr_dis) / 3)
    #a = r_dis * 
    """
    if fr_dis >= limit_dis:
        k = 1
        kd = 0.1
        goal_dis = 50
    
    if fr_dis < limit_dis:
        k = 1
        kd = 0.1
        goal_dis = 20
        """
    angle = k1 * angle_1 + k2 * angle_2 + kd * (r_min - goal_dis)

    if fr_dis < turn_dis:
        pass
        #angle  = -1
    
    angle = np.clip(angle, -1, 1)
    
    speed = 0.5
    print("fr",fr_dis)
    print("min",r_min)
    print(angle)
    
    rc.drive.set_speed_angle(speed, angle)

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    pass # Remove 'pass and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
