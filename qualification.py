import cv2
import RobotAPI as rapi
import numpy as np
import serial
import time
import RPi.GPIO as GPIO

port = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)
robot = rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
G=11
R=15
B=13

GPIO.setup(R,GPIO.OUT)
GPIO.setup(G,GPIO.OUT)
GPIO.setup(B,GPIO.OUT)

GPIO.output(R,GPIO.HIGH)
GPIO.output(G,GPIO.HIGH)
GPIO.output(B,GPIO.HIGH)

message = ""
fps = 0
fps1 = 0
fps_time = 0

#[39, 70 ,84] [111, 255, 164] blue
#[ 0, 87,87] [ 66 ,186 ,166] orange


lowb = np.array([ 91 ,135,   6])
upb = np.array([117, 255, 164])

lowo = np.array([  3 , 79 ,113])
upo = np.array([ 59 ,210, 189])

lowr = np.array([ 0, 89, 47])
upr = np.array([  6,223 ,165])

lowg = np.array([ 64, 200 , 56])
upg = np.array([ 81, 255 ,210])

xl1, yl1 = 0, 200
xr1, yr1 = 620, 200
xp1, yp1 = 280, 400

hb, wb =320, 20
hp, wp =40,80

xl2,yl2 = xl1+wb,yl1+hb
xr2,yr2 = xr1+wb,yr1+hb
xp2,yp2 = xp1+wp,yp1+hp
e_old = 0

sr1 = 0
sr2 = 0


speed = 70


perek=0
color=None


state=0

kp = 0.7
kd = 5
deg=0
u=0
e=0
t111=time.time()
tf=time.time()

ii = ""
flag_start=False
flag_l=False



def pd():
    global  sr1,sr2,e,e_old,deg
    if sr1 > 200:
        sr1 = 200
    if sr2 > 200:
        sr2 = 200

    e = sr2 - sr1

    if -5 < e < 5:
        e = 0

    kp = 0.3
    kd = 3
    u = int(e * kp + (e - e_old) * kd)
    deg = 0 + u
    e_old = e

    if sr1 == 0:
        deg = 30
    if sr2 == 0:
        deg = -30

    if deg > 90:
        deg = 90
    if deg < -90:
        deg = -90

def perecryostok_lyboy():
    global xp1, yp1, xp2, yp2, lowb, upb, lowo, upo, color, perek, t111,flag_l
    datp1 = frame[yp1:yp2, xp1:xp2]
    cv2.rectangle(datp1,(xp1,yp1),(xp2,yp2),(200,100,100),3)
    if color==None or color=="blue":
        hsv1 = cv2.cvtColor(datp1.copy(), cv2.COLOR_BGR2HSV)
        maskd1 = cv2.inRange(hsv1, lowb, upb)    #
        imd1, contoursb, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)


        for contorb1 in contoursb:
            x, y, w, h = cv2.boundingRect(contorb1)
            a1 = cv2.contourArea(contorb1)
            if a1 > 500  and  t111 + 0.9 < time.time():
                color = "blue"
                LED(0,0,1)
                flag_l=True
                # cv2.drawContours(datp1,contoursb,0,(0,0,0),3)
                cv2.rectangle(datp1, (x, y), (x + w, y + h), (255, 0, 0), 2)
                perek+=1
                t111=time.time()


    if color == None or color == "orange":
        hsv2 = cv2.cvtColor(datp1.copy(), cv2.COLOR_BGR2HSV)
        maskd2 = cv2.inRange(hsv2, lowo, upo)  #
        imd1, contourso, hod1 = cv2.findContours(maskd2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        for contoro in contourso:
            x, y, w, h = cv2.boundingRect(contoro)
            a1 = cv2.contourArea(contoro)
            if a1 > 500 and  t111 + 0.6< time.time():
                color = "orange"
                LED(1, 1, 0)
                perek += 1
                flag_l=True

                t111 = time.time()
                # cv2.drawContours(datp1, contourso, 0, (100, 100, 100), 3)
                cv2.rectangle(datp1, (x, y), (x + w, y + h), (0, 100, 255), 2)
    cv2.rectangle(frame, (xp1, yp1), (xp2, yp2), (0, 205, 0), 2)

def black_line_left():
    global xl1, yl1, xl2, yl2, lowb, upb, sr1
    datb1 = frame[yl1:yl2, xl1:xl2]
    dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    gra1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2GRAY)
    _,maskd1 = cv2.threshold(gra1,60,255,cv2.THRESH_BINARY_INV)
    imd1, contoursd1, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    sr1 = 0
    for contorb1 in contoursd1:
        x, y, w, h = cv2.boundingRect(contorb1)
        a1 = cv2.contourArea(contorb1)
        if a1 > 500:
            if sr1 < y + h:
                sr1 = y + h
            cv2.rectangle(datb1, (x, y), (x + w, y + h), (10, 245, 0), 2)
            # cv2.circle(frame, (xl2, sr + 160), 15, (200, 0, 0), 3)

    cv2.rectangle(frame, (xl1, yl1), (xl2, yl2), (0, 0, 255), 2)

def black_line_right():
    global xr1, yr1, xr2, yr2,  sr2
    datb1 = frame[yr1:yr2, xr1:xr2]
    dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    gra1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2GRAY)
    _,maskd1 = cv2.threshold(gra1,60,255,cv2.THRESH_BINARY_INV)
    imd1, contoursd1, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    sr2 = 0

    for contorb1 in contoursd1:
        x, y, w, h = cv2.boundingRect(contorb1)
        a1 = cv2.contourArea(contorb1)
        if a1 > 500:
            if sr2 < y + h:
                sr2 =y + h
            cv2.rectangle(datb1, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # cv2.circle(frame, (xr1, sr + 160), 15, (200, 20, 22), 3)

    cv2.rectangle(frame, (xr1, yr1), (xr2, yr2), (0, 0, 255), 2)

def LED(r,g,b):
    if r==0:
        GPIO.output(R, GPIO.HIGH)
    if r==1:
        GPIO.output(R, GPIO.LOW)

    if g == 0:
        GPIO.output(G, GPIO.HIGH)
    if g == 1:
        GPIO.output(G, GPIO.LOW)

    if b == 0:
        GPIO.output(B, GPIO.HIGH)
    if b == 1:
        GPIO.output(B, GPIO.LOW)


LED(0,0,0)

# d=41,h=35,ds=4,Ds=8,dk=3.8,Dk 7
while 1:
    key = robot.get_key()
    if flag_l==True and t111+0.5<time.time() and state==1:
        LED(0,0,0)
        flag_l=False

    if key != -1:
        pass
        if key == 48:
            state = 0
        elif key == 49:
            state = 1
        elif key == 50:
            state = 2
        elif key == 51:
            state = 3
        elif key == 52:
            state = 4
        elif key == 53:
            state = 5



    frame = robot.get_frame(wait_new_frame=1)

    if state == 0:

        if flag_start == False:
            message = "999999$"
        else:
            message = "200200$"

        if ii=="1":
            state=1
            flag_start=True


    if state==1:# pd


        black_line_left()
        black_line_right()
        pd()
        perecryostok_lyboy()

        if perek==12:
            state=4
            tf = time.time()

        message = str(int(deg) + 200) + str(int(speed) + 200) + '$'

    if state==2:# stop
        pass
         

    if state==4:# finish
        if tf +1.4>time.time():
            black_line_left()
            black_line_right()
            pd()
        else:
            deg=0
            speed=0
            LED(1,1,1)

        message = str(int(deg) + 200) + str(int(speed) + 200) + '$'


    if state==5:# rulevoe

        if key==87:# это W
            speed+=3

        elif key==83:# это S
            speed-=3

        elif key==65:# это A
            deg+=3

        elif key==68:# это D
            deg-=3

        elif key==32:
            speed=0
            deg=0
        elif key==81:
            deg=0
        if speed > 80:
            speed = 80
        if speed < -80:
            speed = -80
        if deg>70:
            deg=60
        if deg <  -70:
            deg = -60

        message = str(int(deg) + 200) + str(int(speed) + 200) + '$'




    fps1 += 1
    if time.time() > fps_time + 1:
        fps_time = time.time()
        fps = fps1
        fps1 = 0




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

    robot.text_to_frame(frame, 'fps = ' + str(fps), 50, 20)

    robot.text_to_frame(frame, 'state = ' + str(state), 50, 40)
    robot.text_to_frame(frame, 'message = ' + str(message), 50, 60)
    robot.text_to_frame(frame, 'perek = ' + str(perek)  +  ' color = ' + str(color), 50, 80)
    robot.text_to_frame(frame, 'sr1 - sr2 = ' + str(sr1) + ' - ' + str(sr2) + ' = ' + str(e), 50, 100)
    robot.set_frame(frame, 40)

    # if key > -1:
    #     print(str(key) + " state "+str(state))