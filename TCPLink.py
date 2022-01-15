import socket
import time
import numpy as np
from numpy.core import uint16


def main():
    # IP and PORT
    TCP_IP = "192.168.4.1"
    TCP_PORT = 8000

    # Receive and send buffer information
    Read_Buffer_size_bytes = 22
    Read_Buffer_size_bits = Read_Buffer_size_bytes * 8

    steer = 10
    speed = 10

    # Establish connection
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    s.settimeout(20)
    print("Connected successfully")
    print(s.getsockname())

    while (1):
        time.sleep(0.2)
        # TODO: Write code for receiving feedback. The current code times out

        # data = s.recv(Read_Buffer_size_bits)
        # if len(data) == Read_Buffer_size_bits:
        #     print(data)

        # Send Commands
        start = uint16(43981)                   # Start (2 bytes)
        steerp = uint16(steer)                  # Steer (2 bytes)
        speedp = uint16(speed)                  # Speed (2 bytes)
        chkSum = (start ^ steerp) ^ speedp
        chkSum = uint16(chkSum)                 # Error detection bytes (2 bytes)
        write_buffer = np.array([start, steerp, speedp, chkSum])
        write_buffer = write_buffer.tobytes()   # Group them into one command as 8 byes

        s.send(write_buffer)                    # Send command


if __name__ == '__main__':
    main()
