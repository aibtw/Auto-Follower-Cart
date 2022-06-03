import numpy as np

steer = 0  # positive steer = clockwise
speed = 0  # positive Speed = move ahead (push the cart)
# Units:
#   Speed : 110 units = 1m / s
#   steer: 40 units = 30 deg / s


def frame_counter(counter, inc: bool, dec: bool):
    if inc and not dec:
        if counter >= 5: counter = 5
        else: counter += 1
    if dec and not inc:
        if counter <= 0: counter = 0
        else: counter -= 1
    return counter


def aruco_control(mode, dist, max_threshold, forward_threshold, back_threshold, x, rot):
    global speed
    global steer
    moving = False
    if forward_threshold < dist < max_threshold and mode == 1:
        move_forward(dist, max_threshold)
        moving = True
    elif dist < back_threshold and mode == 2:
        move_backward(dist)
        moving = True
    else:
        speed = 0

    if mode == 1 and moving:
        if x >= 0.15:
            steer_right(x)
        elif x <= -0.15:
            steer_left(x)
            # print(steer)
        else:
            steer = 0
    elif mode == 2:
        if -30 <= rot <= 30:
            steer = 0
        else:
            steer = int(rot/3)
    return speed, steer


def move_forward(dist, max_dist):
    global speed
    frac = dist/max_dist
    max_speed = 165
    speed = int(max_speed * frac)
    if speed < 60:
        speed = 60
    if speed > 165:
        speed = 165


def move_backward(dist):
    global speed
    max_speed = -110
    speed = -int(-110*np.log10(dist)+220)
    if speed < max_speed:
        speed = -max_speed


def steer_right(x):
    global steer
    max_steer = 70
    # st = np.log10((x*100)+1) * 20
    st = max_steer * x
    if st > max_steer:
        print("[WARNING] Steer exceeding limit. Return to limit")
        st = max_steer
    steer = st


def steer_left(x):
    global steer
    max_steer = 70
    # st = -np.log10((abs(x)*100) + 1) * 20
    st = max_steer * x
    if st < -max_steer:
        print("[WARNING] Steer exceeding limit. Return to limit")
        st = -max_steer
    steer = st


# applying Pythagorean theory
def max_x(angle, adjacent):
    tang = adjacent / np.cos(angle * np.pi / 180)
    opposite = np.sqrt((tang ** 2) - (adjacent ** 2))
    return opposite
