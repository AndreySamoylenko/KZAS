import cv2
import RobotAPI as rapi
import numpy as np
import serial
import time
import  RPi.GPIO as GPIO
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

# [85, 74 ,57] [111 ,255, 116] blue
# [  4 ,104,  99] [ 17, 202 ,255] orange
#[  0 ,105  , 0] [  7 ,255 ,255] red
#[ 72, 121 , 48] [ 93 ,255 ,211]green
#[ 64, 200 , 56] [ 81, 255 ,210]green :)

lowb = np.array([85, 74 ,57])
upb = np.array([111 ,255, 116])

lowo = np.array([  0 ,60,  50])
upo = np.array([ 30, 255 ,255])

lowr = np.array([  0 ,105  , 50])
upr = np.array([  7 ,255 ,255])

lowg = np.array([ 64, 200 , 56])
upg = np.array([ 81, 255 ,210])

xl1, yl1 = 0, 200
xr1, yr1 = 620, 200
xp1, yp1 = 300, 450
xk1, yk1 = 25, 200

hbl, wbl= 280, 20
hbr,wbr=280,20
hp, wp = 20, 40
hk, wk = 240, 590

xl2, yl2 = xl1 + wbl, yl1 + hbl
xr2, yr2 = xr1 + wbr, yr1 + hbr
xp2, yp2 = xp1 + wp, yp1 + hp
xk2, yk2 = xk1 + wk, yk1 + hk

r=0
p=255

sr1 = 0
sr2 = 0
srg = 0
srr = 0


delta=0
speed = 50 #                                                 40
speed_n = int(speed * 0.8)
perek = 0
color = None
color_Final = ""
e_old = 0

state = 0
func=None
kp = 0.7
kd = 5
deg = 0
u = 0
e = 0

v_col=0.15
timer_sp=time.time()
timer_per = time.time()
tf = time.time()
t_colr=time.time()
t_colg=time.time()
t_b=time.time()
vf=1.4

ii = ""
flag_start = False
flag_l=False
danger_flag=False

hg=0
hr=0
x1, y1, w1, h1 =0,0,0,0
x2, y2, w2, h2 =0,0,0,0

cub_list=["",""]

def pd():
    global sr1, sr2, e, e_old, delta, deg,color
    if sr1 > 200:
        sr1 = 200
    if sr2 > 200:
        sr2 = 200


    e = sr2 - sr1

    if -5 < e < 5:
        e = 0

    kp = 0.5
    kd = 5
    u = int(e * kp + (e - e_old) * kd)
    deg =u

    e_old = e

    if sr1 == 0:
        deg = 35
    if sr2 == 0:
        deg = -35

    if deg > 90:
        deg = 90
    if deg < -90:
        deg = -90

    if sr1==0 and sr2==0:
        if color=="orange":
            deg=-45
        else:
            deg=45

def perecryostok_lyboy():
    global xp1, yp1, xp2, yp2, lowb, upb, lowo, upo, color, perek, timer_per, delta,flag_l
    datp1 = frame[yp1:yp2, xp1:xp2]
    cv2.rectangle(datp1, (xp1, yp1), (xp2, yp2), (200, 100, 100), 3)
    if color == None or color == "blue":
        hsv1 = cv2.cvtColor(datp1.copy(), cv2.COLOR_BGR2HSV)
        maskd1 = cv2.inRange(hsv1, lowb, upb)  #
        imd1, contoursb, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        for contorb1 in contoursb:
            x, y, w, h = cv2.boundingRect(contorb1)
            a1 = cv2.contourArea(contorb1)
            if a1 > 50 and timer_per + 0.3 < time.time():
                color = "blue"
                LED(0,0,1)
                flag_l = True

                cv2.rectangle(datp1, (x, y), (x + w, y + h), (255, 0, 0), 2)
                perek += 1
                timer_per = time.time()

    if color == None or color == "orange":
        hsv2 = cv2.cvtColor(datp1.copy(), cv2.COLOR_BGR2HSV)
        maskd2 = cv2.inRange(hsv2, lowo, upo)  #
        imd1, contourso, hod1 = cv2.findContours(maskd2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        for contoro in contourso:
            x, y, w, h = cv2.boundingRect(contoro)
            a1 = cv2.contourArea(contoro)
            if a1 > 50 and timer_per + 0.3 < time.time():
                color = "orange"
                LED(1,1,0)
                flag_l=True
                perek += 1
                cv2.rectangle(datp1, (x, y), (x + w, y + h), (0, 100, 255), 2)
                timer_per = time.time()


    cv2.rectangle(frame, (xp1, yp1), (xp2, yp2), (0, 205, 0), 2)

def cube_R():
    global xk1, yk1, xk2, yk2, lowr, upr, color_Final, perek,  srr,speed,hr,t_colr,x1, y1, w1, h1,timer_sp
    datk1 = frame[yk1:yk2, xk1:xk2]

    hsv1 = cv2.cvtColor(datk1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1, lowr, upr)
    imd1, contoursk, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    max=0

    if len(contoursk)!=0:
        for contork1 in contoursk:
            x, y, w, h = cv2.boundingRect(contork1)
            a1 = cv2.contourArea(contork1)
            if a1 > 200:
                t_colr = time.time()
                if y+h>max:
                    max=y+h
                    srr = x+w
                    hr = y + h
                    x1, y1, w1, h1 =  x, y, w, h
                    timer_sp=time.time()

    else:
        if t_colr+0.1< time.time():
            srr = 0
            hr = 0
            x1, y1, w1, h1 = 0, 0, 0, 0

    cv2.rectangle(datk1, (x1, y1), (x1 + w1, y1 + h1), (0, 0, 255), 2)
    cv2.rectangle(frame, (xk1, yk1), (xk2, yk2), (0, 200,200), 2)

def cube_G():
    global xk1, yk1, xk2, yk2,  lowg, upg, perek, srg,speed,hg,t_colg,x2, y2, w2, h2,timer_sp,color,xl1,xr1,hbl,hbr

    datg1 = frame[yk1:yk2, xk1:xk2]

    hsv1 = cv2.cvtColor(datg1, cv2.COLOR_BGR2HSV)
    maskg1 = cv2.inRange(hsv1, lowg, upg)
    imd1, contoursk, hod1 = cv2.findContours(maskg1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max = 0

    if len(contoursk) != 0:
        for contork1 in contoursk:
            x, y, w, h = cv2.boundingRect(contork1)
            a1 = cv2.contourArea(contork1)
            if a1 > 200:
                t_colg = time.time()
                if y + h > max:
                    max = y + h
                    srg = x
                    hg = y + h
                    x2, y2, w2, h2 = x, y, w, h
                    timer_sp = time.time()



    else:
        if t_colg + 0.1 < time.time():
            srg = 0
            hg = 0
            x2, y2, w2, h2 = 0, 0, 0, 0

    cv2.rectangle(datg1, (x2, y2), (x2 + w2, y2 + h2), (0, 255, 0), 2)

def black_line_left():
    global xl1, yl1, xl2, yl2, lowb, upb, sr1,p,r
    datb1 = frame[yl1:yl2, xl1:xl2]
    dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    gra1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2GRAY)
    _, maskd1 = cv2.threshold(gra1,50 , 255, cv2.THRESH_BINARY_INV)
    imd1, contoursd1, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    sr1 = 0
    for contorb1 in contoursd1:
        x, y, w, h = cv2.boundingRect(contorb1)
        a1 = cv2.contourArea(contorb1)
        if a1 > 600:
            if sr1 < y + h:
                sr1 = y + h
            cv2.rectangle(datb1, (x, y), (x + w, y + h), (10, 245, 0), 2)
            # cv2.circle(frame, (xl2, sr + 160), 15, (200, 0, 0), 3)

    cv2.rectangle(frame, (xl1, yl1), (xl2, yl2), (0, 0, 255), 2)

def black_line_right():
    global xr1, yr1, xr2, yr2, sr2
    datb1 = frame[yr1:yr2, xr1:xr2]
    dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    gra1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2GRAY)
    _, maskd1 = cv2.threshold(gra1, 50, 255, cv2.THRESH_BINARY_INV)
    imd1, contoursd1, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    sr2 = 0

    for contorb1 in contoursd1:
        x, y, w, h = cv2.boundingRect(contorb1)
        a1 = cv2.contourArea(contorb1)
        if a1 > 600:
            if sr2 < y + h:
                sr2 = y + h
            cv2.rectangle(datb1, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # cv2.circle(frame, (xr1, sr + 160), 15, (200, 20, 22), 3)

    cv2.rectangle(frame, (xr1, yr1), (xr2, yr2), (0, 0, 255), 2)


# d=41,h=35,ds=4,Ds=8,dk=3.8,Dk 7
def LED(r,g,b):
    if r==0:
        GPIO.output(R, GPIO.HIGH)
    elif r==1:
        GPIO.output(R, GPIO.LOW)

    if g == 0:
        GPIO.output(G, GPIO.HIGH)
    elif g == 1:
        GPIO.output(G, GPIO.LOW)

    if b == 0:
        GPIO.output(B, GPIO.HIGH)
    elif b == 1:
        GPIO.output(B, GPIO.LOW)


LED(0,0,0)

LED(9,9,9)
while 1:


    key = robot.get_key()
    if flag_l==True and timer_per+0.5<time.time() and state==1:
        LED(0,0,0)
        flag_l=False

    if key != -1:
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

        if ii == "1":
            state = 1
            flag_start = True


    if state == 1:  # pd
        cube_G()
        cube_R()
        black_line_left()
        black_line_right()

        if srg==0 and srr==0:

            if speed < 100:
                speed =int(50+(time.time()-timer_sp)*30)
            pd()
            color_Final = "None"
            speed_n=int(speed*0.95)
            if t_b+0.5<time.time():
                yl1 = 200
                yr1 = 200
                hbl = 280
                hbr = 280
                yl2 = yl1 + hbl
                yr2 = yr1 + hbr
        elif srg!=0 and hg>hr:
            color_Final = "Green"
            t_b = time.time()
            if color=="orange":
                yr1 = 240
                hbr = 240
                yr2 = yr1 + hbr
            else:
                yl1 = 240
                hbl = 240
                yl2 = yl1 + hbl


            e =(240+hg*1.3)-srg
            speed=int(speed_n-hg*0.25)
            if speed < 50:
                speed = 50
            if -5 < e < 5:
                e = 0
            kp = 0.3
            kd = 3
            u = int(e * kp + (e - e_old) * kd)
            deg = 0 + u
            e_old = e

            if deg > 90:
                deg = 90
            if deg < -90:
                deg = -90
        else :
            color_Final = "Red"
            t_b=time.time()
            if color=="orange":
                yr1 = 240
                hbr = 240
                yr2 = yr1 + hbr
            else:
                yl1 = 240
                hbl = 240
                yl2 = yl1 + hbl
            e = (350-hr*1.3)-srr
            speed = int(speed_n-hr*0.3)
            if speed<50:
                speed=50
            if -5 < e < 5:
                e = 0
            kp = 0.3
            kd = 3
            u = int(e * kp + (e - e_old) * kd)
            deg = 0 + u
            e_old = e

            if deg > 90:
                deg = 90
            if deg < -90:
                deg = -90





        perecryostok_lyboy()

        if perek == 2:
            state = 4
            tf = time.time()

        message = str(int(deg) + 200) + str(int(speed) + 200) + '$'
        if len(message)>7:
            message="200250$"

    if state == 2:  # stop
        pass



    if state == 4:  # finish
        if tf + vf> time.time():
            cube_G()
            cube_R()
            black_line_left()
            black_line_right()

            if srg == 0 and srr == 0:
                if speed < 100:
                    speed = int(50 + (time.time() - timer_sp) * 25)
                pd()
                color_Final = "None"
                if t_b + 0.5 < time.time():
                    yl1 = 200
                    yr1 = 200
                    hbl = 280
                    hbr = 280
                    yl2 = yl1 + hbl
                    yr2 = yr1 + hbr
            elif srg != 0 and hg > hr:
                vf=1.8
                color_Final = "Green"
                t_b = time.time()
                if color == "orange":
                    yr1 = 240
                    hbr = 240
                    yr2 = yr1 + hbr
                else:
                    yl1 = 240
                    hbl = 240
                    yl2 = yl1 + hbl

                e = (240 + hg * 1.3) - srg
                speed = int(100 - hg * 0.25)
                if speed < 50:
                    speed = 50
                if -5 < e < 5:
                    e = 0
                kp = 0.3
                kd = 3
                u = int(e * kp + (e - e_old) * kd)
                deg = 0 + u
                e_old = e

                if deg > 90:
                    deg = 90
                if deg < -90:
                    deg = -90
            else:
                vf=1.8
                color_Final = "Red"
                t_b = time.time()
                if color == "orange":
                    yr1 = 240
                    hbr = 240
                    yr2 = yr1 + hbr
                else:
                    yl1 = 240
                    hbl = 240
                    yl2 = yl1 + hbl
                e = (350 - hr * 1.3) - srr
                speed = int(100 - hr * 0.25)
                if speed < 50:
                    speed = 50
                if -5 < e < 5:
                    e = 0
                kp = 0.3
                kd = 3
                u = int(e * kp + (e - e_old) * kd)
                deg = 0 + u
                e_old = e

                if deg > 90:
                    deg = 90
                if deg < -90:
                    deg = -90







        else:
            deg = 0
            speed = 0
            LED(1,1,1)

        message = str(int(deg) + 200) + str(int(speed) + 200) + '$'

    if state == 5:  # rulevoe

        if key == 87:  # это W
            speed += 3

        elif key == 83:  # это S
            speed -= 3

        elif key == 65:  # это A
            deg += 3

        elif key == 68:  # это D
            deg -= 3

        elif key == 82:  # это R
            if GPIO.input(R):
                LED(0 , 9 , 9)
            else:
                LED(1,9,9)

        elif key == 71:  # это G
            if GPIO.input(G):
                LED(9, 0, 9)
            else:
                LED(9, 1, 9)

        elif key == 66:  # это B
            if GPIO.input(B):
                LED(9, 9, 0)
            else:
                LED(9, 9, 1)

        elif key == 32:
            speed = 0
            deg = 0
        if speed > 80:
            speed = 80
        if speed < -80:
            speed = -80
        if deg > 70:
            deg = 60
        if deg < - 70:
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
            try:
                a = str(port.read(), "utf-8")
                if a != '$':
                    ii += a
                else:
                    break
                if t + 0.02 < time.time():
                    break
            except ValueError:
                print("err")

    robot.text_to_frame(frame, 'fps = ' + str(fps), 50, 20)

    robot.text_to_frame(frame, 'state = ' + str(state), 50, 40)
    robot.text_to_frame(frame, 'message = ' + str(message), 50, 60)
    robot.text_to_frame(frame, 'sr1-sr2= ' + str(sr1)+'-' +str(sr2) + ' color = ' + str(color), 50, 80)
    robot.text_to_frame(frame,  'srr ,srg= ' + str(srr) + ' ,' + str(srg) + 'hr ,hg= ' + str(hr) + ' ,' + str(hg),50,100)
    robot.text_to_frame(frame, 'perekryostok ' + str(perek), 50, 120)

    robot.set_frame(frame, 40)

    # if key > -1:
    #     print(str(key) + " state "+str(state))