import socket
import numpy as np
from numpy.core import uint16


def TCP_init():
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
    return s


def send(s, speed, steer):
    # Send Commands
    start = uint16(43981)  # Start (2 bytes)
    steerp = uint16(steer)  # Steer (2 bytes) (Steer is inverted)
    speedp = uint16(speed)  # Speed (2 bytes)
    chkSum = (start ^ steerp) ^ speedp
    chkSum = uint16(chkSum)  # Error detection bytes (2 bytes)
    write_buffer = np.array([start, steerp, speedp, chkSum])
    write_buffer = write_buffer.tobytes()  # Group them into one command as 8 byes
    s.send(write_buffer)  # Send command


def receive(s, debug=False):
    feedback = s.recv(22)
    if len(feedback) >= 22:
        # print("message from server:", feedback.decode("UTF-8"))   # For debugging, if sent message is UTF-8
        # print("message from server:", feedback)                   # For debugging, if sent message is numeric
        header = feedback[0:2]
        cmd1 = feedback[2:4]  # Current steer
        cmd2 = feedback[4:6]  # Current speed
        spdR = feedback[6:8]  # Motor speed R
        spdL = feedback[8:10]  # Motor speed L
        cntR = feedback[10:12]  # Wheels encoder R
        cntL = feedback[12:14]  # Wheels encoder L
        batV = feedback[14:16]  # Battery voltage
        temp = feedback[16:18]  # Temperature
        cmdLED = feedback[18:20]  # LED
        chkSum = feedback[20:22]  # Error detection
        if debug:
            # TODO: Cast all the byte-type variables into the appropriate type (int16-uint16)
            print(
                f"speed: {int.from_bytes(cmd2, byteorder='little')} | temp: {int.from_bytes(temp, byteorder='little')} | "
                f"voltage: {int.from_bytes(batV, byteorder='little')}")
            print(f"temp: {int.from_bytes(temp, byteorder='little')}")
            print(f"voltage: {int.from_bytes(batV, byteorder='little')}")
