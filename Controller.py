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


def aruco_control(dist, max_threshold, forward_threshold, back_threshold, x):
    global speed
    global steer
    if forward_threshold < dist < max_threshold:
        move_forward(dist, max_threshold)
    elif dist < back_threshold:
        move_backward(dist, max_threshold)
    else:
        speed = 0

    if x >= 0.15:
        steer_right(x)
    elif x <= -0.15:
        steer_left(x)
    else:
        steer = 0
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


def move_backward(dist, max_dist):
    global speed
    speed = -30


def steer_right(x):
    global steer
    max_steer = 70
    # st = np.log10((x*100)+1) * 20
    st = max_steer * x
    if st >= max_steer:
        print("[WARNING] Steer exceeding limit. Return to limit")
        st = max_steer
    steer = st


def steer_left(x):
    global steer
    max_steer = 70
    # st = -np.log10((abs(x)*100) + 1) * 20
    st = max_steer * x
    if st <= -max_steer:
        print("[WARNING] Steer exceeding limit. Return to limit")
        st = -max_steer
    steer = st


# applying Pythagorean theory
def max_x(angle, adjacent):
    tang = adjacent / np.cos(angle * np.pi / 180)
    opposite = np.sqrt((tang ** 2) - (adjacent ** 2))
    return opposite
