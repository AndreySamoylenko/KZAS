# coding=utf-8
import cv2
import numpy as np
import RobotAPI as rapi
robot = rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)

HUE_MIN = 0
HUE_MAX = 180
SAT_MIN = 0
SAT_MAX = 255
VAL_MIN = 0
VAL_MAX = 255
trackbar_vals = {'min': np.array([HUE_MIN, SAT_MIN, VAL_MIN]),
                 'max': np.array([HUE_MAX, SAT_MAX, VAL_MAX])}

flag_cam = True
flag_key = True

frame = robot.get_frame(wait_new_frame=1)
frame1 = frame
while 1:
    key = robot.get_key()
    if key != -1:
        #print(key)
        if key == 32:
            flag_cam = False
        if key == 27:
            flag_cam = True

        if key == 81 and flag_key == True and trackbar_vals['min'][0]<180:
            trackbar_vals['min'][0] += 1
        if key == 65 and flag_key == True and trackbar_vals['min'][0]>0:
            trackbar_vals['min'][0] -= 1
        if key == 87 and flag_key == True and trackbar_vals['max'][0]<180:
            trackbar_vals['max'][0] += 1
        if key == 83 and flag_key == True and trackbar_vals['max'][0]>0:
            trackbar_vals['max'][0] -= 1


        if key == 69 and flag_key == True and trackbar_vals['min'][1]<255:
            trackbar_vals['min'][1] += 1
        if key == 68 and flag_key == True and trackbar_vals['min'][1]>0:
            trackbar_vals['min'][1] -= 1
        if key == 82 and flag_key == True and trackbar_vals['max'][1]<255:
            trackbar_vals['max'][1] += 1
        if key == 70 and flag_key == True and trackbar_vals['max'][1]>0:
            trackbar_vals['max'][1] -= 1


        if key == 84 and flag_key == True and trackbar_vals['min'][2]<255:
            trackbar_vals['min'][2] += 1
        if key == 71 and flag_key == True and trackbar_vals['min'][2]>0:
            trackbar_vals['min'][2] -= 1
        if key == 89 and flag_key == True and trackbar_vals['max'][2]<255:
            trackbar_vals['max'][2] += 1
        if key == 72 and flag_key == True and trackbar_vals['max'][2]>0:
            trackbar_vals['max'][2] -= 1


        flag_key = False
        print(trackbar_vals['min'], trackbar_vals['max'])
    else:
        flag_key = True


    if flag_cam:
        frame = robot.get_frame(wait_new_frame=1)
        frame = cv2.flip(frame, 1)


    imageHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(imageHSV, trackbar_vals['min'], trackbar_vals['max'])
    frame1 = cv2.bitwise_or(frame, frame, mask=mask)
    robot.set_frame(frame1,40)
    #print( trackbar_vals['min'], trackbar_vals['max'])

