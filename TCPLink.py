import socket
import time
import numpy as np
from pynput import keyboard
from numpy.core import uint16

steer = 10
speed = 10

def main():
    print("Program Starting!")

    # IP and PORT of the ESP server
    TCP_IP, TCP_PORT = "192.168.4.1", 8000

    # Receive and send buffer information
    Read_Buffer_size_bytes = 22


    # Establish connection
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    s.settimeout(20)
    print("Connected successfully")
    print(s.getsockname())

    while True:
        time.sleep(0.2)

        # Send Commands
        start = uint16(43981)                   # Start (2 bytes)
        steerp = uint16(steer)                  # Steer (2 bytes)
        speedp = uint16(speed)                  # Speed (2 bytes)
        chkSum = (start ^ steerp) ^ speedp
        chkSum = uint16(chkSum)                 # Error detection bytes (2 bytes)
        write_buffer = np.array([start, steerp, speedp, chkSum])
        write_buffer = write_buffer.tobytes()   # Group them into one command as 8 byes
        s.send(write_buffer)                    # Send command

        # Receive Feedback
        # TODO: Determine whether recv function accepts only power-of-2 values (i.e. 32) or it can accept 22 bytes
        feedback = s.recv(22)
        if len(feedback) >= Read_Buffer_size_bytes:
            # print("message from server:", feedback.decode("UTF-8"))   # For debugging, if sent message is UTF-8
            # print("message from server:", feedback)                   # For debugging, if sent message is numeric
            header = feedback[0:2]
            cmd1 = feedback[2:4]                                        # Current steer
            cmd2 = feedback[4:6]                                        # Current speed
            spdR = feedback[6:8]                                        # Motor speed R
            spdL = feedback[8:10]                                       # Motor speed L
            cntR = feedback[10:12]                                      # Wheels encoder R
            cntL = feedback[12:14]                                      # Wheels encoder L
            batV = feedback[14:16]                                      # Battery voltage
            temp = feedback[16:18]                                      # Temperature
            cmdLED = feedback[18:20]                                    # LED
            chkSum = feedback[20:22]                                    # Error detection

            # TODO: Cast all the byte-type variables into the appropriate type (int16-uint16)


def on_press(key):
    global speed
    global steer
    if key == keyboard.Key.up:
        print('up')
        speed=speed+1
        print(speed)
        if speed>50:
            speed = 50
    if key == keyboard.Key.down:
        print('down')
        speed=speed-1
        print(speed)
        if speed<-50:
            speed = -50
    if key == keyboard.Key.left:
        print('left')

    if key == keyboard.Key.right:
        print('right')


def on_release(key):
    # print('{0} released'.format(
    #     key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False


with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()
print(speed)
print(steer)

if __name__ == '__main__':
    main()


