import numpy as np

"""
@author: Ahmed Patwa
Last updated on 8/6/2022
"""

# Defining global speed and steer and setting it to 0
steer = 0  # positive steer = clockwise
speed = 0  # positive Speed = move ahead (in the direction of the camera)
# Units:
#   Speed : 110 units = 1m / s
#   steer: 40 units = 30 deg / s


def frame_counter(counter, inc: bool, dec: bool):
    """
    A function responsible for counting number of frames that had an aruco detected in it.
    The value (counter) is increased on each detection, and decreased on each miss.

    :param counter: integer counter of number of frames that had an aruco detected in it.
    :param inc: boolean flag. If inc is 1 then increment counter
    :param dec: boolean flag. If dec is 1 then decrement counter
    :return: counter
    """
    if inc and not dec:
        if counter >= 5: counter = 5
        else: counter += 1
    if dec and not inc:
        if counter <= 0: counter = 0
        else: counter -= 1
    return counter


def aruco_control(mode, dist, max_threshold, forward_threshold, back_threshold, x, rot, prev_speed):
    """
    A function to manage controlling the robot based on the output from pose estimation of Aruco markers

    :param mode: Enum type. follow: mode = 1 | push: mode = 2.
    :param dist: The current distance between Aruco marker and the camera.
    :param max_threshold: The maximum distance at which the robot won't follow the user in follow mode.
    :param forward_threshold: The minimum distance at which the robot starts following the user in follow mode.
    :param back_threshold: the maximum distance between Aruco and camera, at which the robot can be pushed in push mode.
    :param x: Percentage of the position of the Aruco marker along the x axis.
    :param rot: Rotation along the z axis of the aruco marker
    :param prev_speed: The speed of the robot in the previous frame (not used)
    :return: int speed, int steer
    """
    global speed
    global steer

    moving = False                                              # the state of the robot. Set to True if speed > 0

    # ------------------------------------------------- Speed section -------------------------------------------------#
    if forward_threshold < dist < max_threshold and mode == 1:  # if in follow mode, and within boundaries of detection
        move_forward(dist, max_threshold)
        moving = True
    elif dist < back_threshold and mode == 2:                   # else, if it is push mode and within boundaries
        move_backward(dist)
        moving = True
    else:
        speed = 0                                               # otherwise, the speed is set to 0

    # ------------------------------------------------- steer section -------------------------------------------------#
    if mode == 1 and moving:                            # follow mode and the robot is moving (no steer if idle)
        steer_bound = 0.25 if dist < 130 else 0.20      # this is the safe area where the robot won't steer.
        if x >= steer_bound:
            steer_right(x)
        elif x <= -steer_bound:
            steer_left(x)
        else:
            steer = 0
    elif mode == 2:                                     # if it is in push mode, steer based on rot, not dist.
        if -30 <= rot <= 30:                            # this is the safe angle where the robot won't steer.
            steer = 0
        else:
            steer = int(rot/3)
    return speed, steer


def move_forward(dist, max_dist):
    """
    a function for following mode, which calculates adaptive speed value based on current distance and maximum distance
    :param dist: The current distance of the Aruco marker from the camera
    :param max_dist: The maximum distance at which the robot won't follow the user
    """
    global speed
    frac = dist/max_dist                # Calculating the fraction of the current distance to the maximum distance
    max_speed = 165                     # The maximum possible speed for the robot
    speed = int(max_speed * frac)       # The equation to calculate speed

    # Make sure the speed is within limits of minumum and maximum speed
    if speed < 60:
        speed = 60
    if speed > max_speed:
        speed = max_speed


def move_backward(dist):
    """
    A function for push mode, which calculates an adaptive speed based on the current distance
    :param dist: The current distance between Aruco marker and the camera
    :return:
    """
    global speed
    max_speed = -110                        # Maximum possible speed for the robot
    speed = -int(-110*np.log10(dist)+220)   # Behavior of log function makes speed increase when we come closer,
                                            # and speed will increase if we go further away.
    # Make sure the speed is within limits
    if speed > 0: speed = 0
    if speed < max_speed:
        speed = max_speed


# TODO: steer_right and steer_left functions are actually duplicate! clean the code and put them in one function
def steer_right(x):
    """
    A function to calculate an adaptive steer value to the right
    :param x: Percentage of the position of the Aruco marker along the x axis.
    :return:
    """
    global steer
    max_steer = 50          # Maximum possible steer value
    st = max_steer * x      # the simple equation to calculate steering value based on the percentage and maximum steer
    # make sure the value is within boundaries
    if st > max_steer:
        st = max_steer
    steer = st


def steer_left(x):
    """
    A function to calculate an adaptive steer value to the left
    :param x: Percentage of the position of the Aruco marker along the x axis.
    :return:
    """
    global steer
    max_steer = 50      # Maximum possible steer value
    st = max_steer * x  # the simple equation to calculate steering value based on the percentage and maximum steer
    # make sure the value is within boundaries
    if st < -max_steer:
        st = -max_steer
    steer = st


# applying Pythagorean theory
def max_x(angle, adjacent):
    """
    This function applies Pythagorean theory on the field of view of the camera, to obtain the max distance in x axis
    of the field of view (refer to the reports of the project for explanation)
    :param angle: Field of view angle
    :param adjacent: adjacent side of the triangle
    :return: opposite
    """
    tang = adjacent / np.cos(angle * np.pi / 180)
    opposite = np.sqrt((tang ** 2) - (adjacent ** 2))
    return opposite
