# initialize
"""
import sys
import math
import copy
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import ipywidgets as widgets
import statistics
from nptyping import NDArray
from typing import Any, Tuple, List, Optional
from enum import Enum
import matplotlib.pyplot as plt
sys.path.insert(0,'../../library')
"""
# initialize
import sys
import math
import copy
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import ipywidgets as widgets
import statistics
from nptyping import NDArray
from typing import Any, Tuple, List, Optional
from enum import Enum
import matplotlib.pyplot as plt

sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

# variables
angle = 0.0
speed = 0.0
MIN_CONTOUR_AREA = 100
angle_values = []
image = None

# A crop window for the floor directly in front of the car
# CROP_FLOOR = ((150, 0), (210,320))
CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

# TODO Part 1: Determine the HSV color threshold pairs for GREEN and RED
# Colors, stored as a pair (hsv_min, hsv_max) Hint: Lab E!

# BLUE = ((0, 105, 129), (179, 210, 255))
# GREEN = ((35, 50, 50), (80, 255, 255))  # The HSV range for the color green
# RED = ((0, 50, 50), (10, 255,255))  # The HSV range for the color red

BLUE = ((90, 50, 50), (130, 255, 255))  # The HSV range for the color blue
GREEN = ((35, 50, 50), (80, 255, 255))  # The HSV range for the color green
RED = ((0, 50, 50), (10, 255, 255))

# Color priority: Red >> Green >> Blue

prev_id = None
first_id = None

wall_follow_speed = 0.0
wall_follow_integral = 0.0
wall_follow_pre_error = 0.0
wall_follow_kp = 0.02
wall_follow_ki = 0
wall_follow_kd = 0.0001

line_follow_speed = 0.0
line_follow_contour_center = None
line_follow_contour_area = 0
line_follow_integral = 0.0
line_follow_pre_error = 0.0
line_follow_kp = 1
line_follow_ki = 0
line_follow_kd = 0.2
COLOR_PRIORITY = (RED, GREEN, BLUE)


# color


# functions
def pixel_regression(area_cone, area_cone_min):
    a_fit = 5749.1
    b_fit = -2.459
    if area_cone >= area_cone_min:
        dist = a_fit / area_cone ** 0.5 + b_fit
    else:
        dist = 0
    return dist


def update_contour():
    global contour_center
    global contour_area
    global image
    global CROP_FLOOR
    # print(image.shape)
    # print(CROP_FLOOR)
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Crop the image to the floor directly in front of the car
        image2 = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        for i, color in enumerate(COLOR_PRIORITY):
            contours = rc_utils.find_contours(image2, *color)
            tmp_contour_area = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
            if tmp_contour_area is not None:
                contour_area = tmp_contour_area
                contour_center = rc_utils.get_contour_center(contour_area)
                break
        else:
            # if any color's contours is None, variables become init value
            contour_center = None
            contour_area = 0

        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found

        # Display the image to the screen


def wall_follow():
    global wall_follow_integral
    global wall_follow_pre_error
    global angle
    global speed

    """
    scan = rc.lidar.get_samples()
    right_front_dist = rc_utils.get_lidar_average_distance(scan, 52, 38)
    right_rear_dist = rc_utils.get_lidar_average_distance(scan, 128, 38)
    right_error = right_front_dist - right_rear_dist
    error = right_error
    wall_follow_integral += error
    derivative = error - wall_follow_pre_error
    before = wall_follow_kp * error + wall_follow_ki * wall_follow_integral + wall_follow_kd * derivative
    wall_follow_pre_error = error
    angle = np.clip(before,-1,1)
    """
    scan = rc.lidar.get_samples()

    # left_front_dist = rc_utils.get_lidar_average_distance(scan, -52, 38)
    right_front_dist = rc_utils.get_lidar_average_distance(scan, 52, 38)
    # left_rear_dist = rc_utils.get_lidar_average_distance(scan, -128, 38)
    right_rear_dist = rc_utils.get_lidar_average_distance(scan, 128, 38)
    # left_error = left_front_dist - left_rear_dist
    right_error = right_front_dist - right_rear_dist
    # error =  right_front_dist - right_rear_dist - left_front_dist + left_rear_dist

    """
    print("lf: ",left_front_dist," rf: ",right_front_dist," lr: ",left_rear_dist," rr: ",right_rear_dist)
    error = 0
    if left_error >= right_error:
        error = left_error * -1
    else:
        error = right_error
    """
    print(" right_error: ", right_error)
    error = right_error
    wall_follow_integral += error
    derivative = error - wall_follow_pre_error
    before = wall_follow_kp * error + wall_follow_ki * wall_follow_integral + wall_follow_kd * derivative
    print(f"error_before:{before}")
    wall_follow_pre_error = error

    if rc.controller.is_down(rc.controller.Button.X):
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

        angle = before
        """
        if right_front_dist == 0 or right_rear_dist == 0 or right_front_dist >= 400:
            angle = 2 * kp * (- left_front_dist + left_rear_dist)
            mode = 2
        if left_front_dist == 0 or left_rear_dist == 0 or left_front_dist >= 400:
            angle = 2 * kp * (right_front_dist - right_rear_dist)
            mode = 3
        """

        # angle_values.append(angle)

        angle = np.clip(angle, -1, 1)
        speed = 0.5
        rc.drive.set_speed_angle(speed, angle)
        angle_values.append(angle)


def line_follow():
    """
    global angle
    global speed
    global angle
    global line_follow_pre_error
    global line_follow_integral
    global line_follow_contour_center
    global line_follow_contour_area
    speed = 0
    image = rc.camera.get_color_image()
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        image = rc_utils.crop(image,CROP_FLOOR[0],CROP_FLOOR[1])
        contours = rc_utils.find_contours(image,BLUE)
        tmp_contour_area = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
        if tmp_contour_area is not None:
            contour_area = tmp_contour_area
            contour_center = rc_utils.get_contour_center(contour_area)
        else:
            contour_center = None
            contour_area = 0
    if contour_center is not None:
        error = rc_utils.remap_range(contour_center[1], 0, 320, -1, 1)
    else:
        error = 0.0
    line_follow_integral += 0
    if contour_center is None:
        line_follow_integral = 0
    derivative = error - line_follow_pre_error
    angle = line_follow_kp * error + line_follow_ki * line_follow_integral + line_follow_kd * derivative
    line_follow_pre_error = error
    angle = np.clip(angle,-1,1)
    speed = -0.1
    rc.drive.set_speed_angle(line_follow_speed, angle)
    """
    global image
    # update_contour()
    global speed
    global angle
    global line_follow_pre_error
    global line_follow_integral
    if contour_center is not None:
        # error = contour_center[1] - rc.camera.get_width() / 2

        error = rc_utils.remap_range(contour_center[1], 0, 640, -1, 1)
    else:
        error = 0.0
        print("NO CONTOUR")
    line_follow_integral += error
    if contour_center is None:
        integal = 0
    derivative = error - line_follow_pre_error
    angle = line_follow_kp * error + line_follow_ki * line_follow_integral + line_follow_kd * derivative
    # angle = kp * error
    print("angle:", angle)
    line_follow_pre_error = error
    angle_values.append(angle)
    angle = np.clip(angle, -1, 1)

    # Use the triggers to control the car's speed
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt

    if rc.controller.is_down(rc.controller.Button.X):
        print("X button down")
        speed = 0.5
        rc.drive.set_speed_angle(speed, angle)


def detect_marker():
    global image
    global ar_markers
    global prev_id
    ar_markers = cv.aruco.detectMarkers(image, cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250),
                                        parameters=cv.aruco.DetectorParameters_create())
    if ar_markers is not None:
        if ar_markers[1] is not None:
            # print("BIngchilling")
            ids = ar_markers[1]
            # first_id = ids[0][0]
            print("ARARARAR", ar_markers[0][0][0])
            corner = ar_markers[0][0][0]
            # square = abs((corner[0][0]- corner[2][0]) * (corner[0][1]- corner[2][1]))
            square = (abs((corner[2][0] - corner[1][0]) * (corner[0][1] - corner[1][1])
                          - (corner[0][0] - corner[1][0]) * (corner[2][1] - corner[1][1]) +
                          abs((corner[0][1] - corner[3][0]) * (corner[2][1] - corner[3][1])
                              - (corner[2][0] - corner[3][0]) * (corner[0][1] - corner[3][1])))) / 2
            print(square)
            if square > 5500:
                print("ukraine")
                first_id = ids[0][0]
            else:
                first_id = prev_id
                print("russia")

        else:
            # print("bingCHILLING")
            first_id = prev_id
    else:
        first_id = prev_id
    return first_id

    prev_id = first_id
    if ar_markers is None:
        print("NO MARKER")


# start
def start():
    pass


# update
def update():
    global prev_id
    global first_id
    global ar_markers
    global image
    image = rc.camera.get_color_image()
    update_contour()
    """
    ar_markers = cv.aruco.detectMarkers(image, cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250),
                                        parameters=cv.aruco.DetectorParameters_create())
    if ar_markers is not None:
        if ar_markers[1] is not None:
            ids = ar_markers[1]
            first_id = ids[0][0]
        else:
            first_id = prev_id
    else:
        first_id = prev_id
    """

    first_id = detect_marker()

    # print(f"first id :{first_id}")

    if image is None:
        print("めいよー")
    if first_id == 32:
        print("mode 1")
        wall_follow()
    # elif first_id == 32:
    elif first_id == 1:
        print("mode 2")
        line_follow()
        print(contour_center)
    elif rc.controller.is_down(rc.controller.Button.X):
        print("mode 3")
        speed2 = 0.5
        print("你好")
        rc.drive.set_speed_angle(speed2, 0)
    elif rc.controller.is_down(rc.controller.Button.Y):
        print("mode 4")
        speed2 = -0.5
        print("Wazzup")
        rc.drive.set_speed_angle(speed2, 0)

    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:, contour_area")

    prev_id = first_id
    if ar_markers is None:
        print("NO MARKER")


# update slow
def update_slow():
    pass


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
    plt.plot(angle_values, color='blue')
    plt.xlabel('Time')
    plt.ylabel('Angle')
    plt.title('Angle over Time')
    plt.show()