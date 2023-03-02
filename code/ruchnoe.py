import cv2
import RobotAPI as rapi
import numpy as np
import serial
import time

port = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)
robot = rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)

message = ""
fps = 0
fps1 = 0
fps_time = 0

state = 5
speed = 0
deg = 0
ii = '0'

i = 0
s = 0

message = str('999999$')
while ii == '0':
    port.write(message.encode("utf-8"))
    if port.in_waiting > 0:
        ii = ""
        t = time.time()
        while 1:
            a = str(port.read(), "utf-8")
            if a != '$':
                ii += a
            else:
                break
            if t + 0.02 < time.time():
                break

while 1:
    frame = robot.get_frame(wait_new_frame=1)
    key = robot.get_key()
    if state == 5:  # rulevoe
        if key == -1:
            i += 1
            if i > 10:
                if speed > 8:
                    speed -= 1
                elif speed < -8:
                    speed += 1
                i = 0
        elif key == 87:  # это W
            speed = 80
            i = 0

        elif key == 83:  # это S
            speed = -50
            i = 0

        elif key == 65:  # это A
            deg += 3

        elif key == 68:  # это D
            deg -= 3

        elif key == 32:
            speed = 0
            deg = 0
        if speed > 80 and key == -1:
            speed = 80
        if speed < -50 and key == -1:
            speed = -50
        if deg > 40 and key == -1:
            deg = 40
        if deg < - 40 and key == -1:
            deg = -40

        message = str(int(deg) + 200) + str(int(speed) + 200) + '$'
        port.write(message.encode("utf-8"))
    robot.text_to_frame(frame, 'fps = ' + str(fps), 50, 20)

    robot.text_to_frame(frame, 'message = ' + str(message), 50, 60)

    robot.set_frame(frame, 40)
