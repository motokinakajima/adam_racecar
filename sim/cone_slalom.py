"""
MIT BWSI Autonomous RACECAR
MIT License
bwsix RC101 - Fall 2023

File Name: lab_f.py

Title: Lab F - Line Follower

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: Write a script to enable fully autonomous behavior from the RACECAR. The
RACECAR should automatically identify the color of a line it sees, then drive on the
center of the line throughout the obstacle course. The RACECAR should also identify
color changes, following colors with higher priority than others. Complete the lines 
of code under the #TODO indicators to complete the lab.

Expected Outcome: When the user runs the script, they are able to control the RACECAR
using the following keys:
- When the right trigger is pressed, the RACECAR moves forward at full speed
- When the left trigger is pressed, the RACECAR, moves backwards at full speed
- The angle of the RACECAR should only be controlled by the center of the line contour
- The RACECAR sees the color RED as the highest priority, then GREEN, then BLUE
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from nptyping import NDArray
from typing import Any, Tuple, List, Optional

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, "../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 50

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

# TODO Part 1: Determine the HSV color threshold pairs for GREEN and RED
# Colors, stored as a pair (hsv_min, hsv_max) Hint: Lab E!
BLUE = ((110, 50, 50), (130, 255, 255))  # The HSV range for the color blue
GREEN = ((35, 50, 50), (80, 255, 255))  # The HSV range for the color green
RED = ((160,100,20),(179,255,255))  # The HSV range for the color red

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

integral_dif = 0
spent_time = 0
manual = True

########################################################################################
# Functions
########################################################################################

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
def update_contour(color):
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:

        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found

        # Get each color's contours        
        contours = rc_utils.find_contours(image, color[0],color[1])
        tmp_contour_area = rc_utils.get_largest_contour(contours)
        if tmp_contour_area is not None:
            contour_area = rc_utils.get_contour_area(tmp_contour_area)
            contour_center = rc_utils.get_contour_center(tmp_contour_area)
        
        else:
            # if any color's contours is None, variables become init value
            contour_center = None
            contour_area = 0

        # Display the image to the screen
        #rc.display.show_color_image(image)

def cone_distance(area_cone, area_cone_min):
    a_fit = 5749.1
    b_fit = -2.459
    if area_cone >= area_cone_min:
        dist = a_fit / area_cone**0.5 + b_fit
    else:
        dist = 0
    return dist

def cone_slalom(y, danger_distance, safe_distance, yellow_arrow):
 # y = コーンまでの距離
 # danger_distance = コーンと近づきすぎた距離を定義する数値
 # safe_distance = コーンと適切な距離にあることを定義する数値

    if y <= danger_distance:
        yellow_arrow -= 100
        angle = yellow_arrow*(0.6/320)
        #「0.6」は要調整
    elif y <= safe_distance:
        angle = yellow_arrow*(0.4/320)
        #「0.3」は要調整
    else:
        angle = 0
    if angle >= 1.0:
        angle = 1.0
    elif angle <= -1.0:
        angle = -1.0
    return angle

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle

    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(
        ">> Lab 2A - Color Image Line Following\n"
        "\n"
        "Controls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
        "   A button = print current speed and angle\n"
        "   B button = print contour center and area"
    )

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global integral_dif, spent_time, manual

    # Search for contours in the current color image

    if rc.controller.is_down(rc.controller.Button.A):
        manual = True
    
    if rc.controller.is_down(rc.controller.Button.B):
        manual = False

    # TODO Part 3: Determine the angle that the RACECAR should receive based on the current 
    # position of the center of line contour on the screen. Hint: The RACECAR should drive in
    # a direction that moves the line back to the center of the screen.

    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    if manual:
        (angle, speed) = rc.controller.get_joystick(rc.controller.Joystick.LEFT)

    if not manual:
        (angle, speed) = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
        speed *= 0.5
        blue_area = 0
        blue_center = None
        red_area = 0
        red_center = None
        red_is_nearer = True
        update_contour(BLUE)
        blue_area = contour_area
        blue_center = contour_center
        update_contour(RED)
        red_area = contour_area
        red_center = contour_center
        y = 0
        gap = 290
        danger_zone = 60
        safe_zone = 200
        if not red_area == None:
            print("red found!!")
        if not blue_area == None:
            print("blue found!!")
        print("red: ",red_area," blue:",blue_area)
        if not red_center == None:
            print("red[0]:",red_center[0],"red[1]:",red_center[1])
        if not blue_center == None:
            print("blue[0]:",blue_center[0],"blue[1]:",blue_center[1])

        if not (red_area == None and not blue_area == None):
            if red_area > blue_area:
                red_is_nearer = True
                y = cone_distance(red_area,100)
                print("distance: ",y)
                if not red_center == None:
                    print("angle: ",cone_slalom(y,danger_zone,safe_zone,red_center[1] - (320 - gap)))
                    angle = cone_slalom(y,danger_zone,safe_zone,red_center[1] - (320 - gap))
                    print("Center:", red_center, "Area:", red_area)
            else:
                red_is_nearer = False
                y = cone_distance(blue_area,100)
                print("distance: ",y)
                if not blue_center == None:
                    print("angle: ",cone_slalom(y,danger_zone,safe_zone,blue_center[1] - (320 + gap)))
                    angle = cone_slalom(y,danger_zone,safe_zone,blue_center[1] - (320 + gap))
                    print("Center:", blue_center, "Area:", blue_area)
        else:
            if blue_area == None and not red_area == None:
                red_is_nearer = True
                y = cone_distance(red_area,100)
                print("distance: ",y)
                if not red_center == None:
                    print("angle: ",cone_slalom(y,danger_zone,safe_zone,red_center[1] - (320 - gap)))
                    angle = cone_slalom(y,danger_zone,safe_zone,red_center[1] - (320 - gap))
                    print("Center:", red_center, "Area:", red_area)
            elif not blue_area == None and not red_area == None:
                red_is_nearer = False
                y = cone_distance(blue_area,100)
                print("distance: ",y)
                if not blue_center == None:
                    print("angle: ",cone_slalom(y,danger_zone,safe_zone,blue_center[1] - (320 + gap)))
                    angle = cone_slalom(y,danger_zone,safe_zone,blue_center[1] - (320 + gap))
                    print("Center:", blue_center, "Area:", blue_area)
    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)
    
    print("=======================================")
    print("")

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    """
    # Print a line of ascii text denoting the contour area and x-position
    if rc.camera.get_color_image() is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area))

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_area))
    """


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
