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
R=11
B=15
G=13
timerCVET=time.time()
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

#[ 91 ,135,   6] [117, 255, 164] blue
#[ 6, 78 ,86] [ 43, 218 ,235] orange
#[  0 ,136,  76] [  7,234 ,240] red
#[ 72, 121 , 48] [ 93 ,255 ,211]green
#[ 28, 176 , 74] [ 52, 255, 236] yellow

lowb = np.array([ 91 ,135,   6])
upb = np.array([117, 255, 164])

lowo = np.array([ 6, 78 ,86])
upo = np.array( [ 43, 218 ,235])

lowr = np.array([  0 ,136,  76])
upr = np.array([  7,234 ,240])

lowg = np.array([ 64, 200 , 56])
upg = np.array([ 81, 255 ,210])

lowy= np.array([ 28, 176 , 74])
upy=np.array([ 52, 255, 236])

xl1, yl1 = 0, 200
xr1, yr1 = 620, 200
xp1, yp1 = 300, 400
xk1, yk1 = 35,160

hbl, wbl= 280, 20
hbr,wbr=280,20
hp, wp = 25, 40
hk, wk = 220, 580


xl2, yl2 = xl1 + wbl, yl1 + hbl
xr2, yr2 = xr1 + wbr, yr1 + hbr
xp2, yp2 = xp1 + wp, yp1 + hp
xk2, yk2 = xk1 + wk, yk1 + hk



sr1 = 0
sr2 = 0
srg = 0
srr = 0
sry=0


delta=0
speed = 70 #                                                 40
speed_n = int(speed * 0.8)
perek = 0
color = None
color_Final = ""
text=""
e_old = 0

state = 0
func=None
kp = 0.7
kd = 5
deg = 0
u = 0
e = 0
Bank=[]
v_col=0.15
timer_sp=time.time()
timer_per = time.time()
tf = time.time()
t_colr=time.time()
t_colg=time.time()
t_coly=time.time()
t_b=time.time()
ts=time.time()
vf=1.3

ii = ""
flag_start = False
flag_l=False
col_g=False
flag_g=True
col_r=False
flag_r=True
col_y=False
flag_y=True

list=[[],[]]
cub=0

hg=0
hr=0
hy=0
x1, y1, w1, h1 =0,0,0,0
x2, y2, w2, h2 =0,0,0,0

# for i in range(100):
#     for j in range(2):
#         list[i][j]=None
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

    kp = 0.3
    kd = 3
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
    global xp1, yp1, xp2, yp2, lowb, upb, lowo, upo, color, perek, timer_per, delta,flag_l,cub
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
                timer=time.time()
                cub=0
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
                timer = time.time()
                cub = 0
                cv2.rectangle(datp1, (x, y), (x + w, y + h), (0, 100, 255), 2)
                timer_per = time.time()


    cv2.rectangle(frame, (xp1, yp1), (xp2, yp2), (0, 205, 0), 2)

def cube_Y():
    global xk1, yk1, xk2, yk2, lowy, upy, color_Final, perek,  sry,speed,hy,t_coly,x1, y1, w1, h1,timer_sp,cube,flag,\
        timerCVET
    datk1 = frame[yk1:yk2, xk1:xk2]

    hsv1 = cv2.cvtColor(datk1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1, lowy, upy)
    imd1, contoursk, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    max=0

    if len(contoursk)!=0:
        timerCVET = time.time()
        for contork1 in contoursk:
            x, y, w, h = cv2.boundingRect(contork1)
            a1 = cv2.contourArea(contork1)
            if a1 > 300:
                t_coly= time.time()
                if y+h>max:
                    max=y+h
                    sry = x+w
                    hy = y + h
                    x1, y1, w1, h1 =  x, y, w, h
                    timer_sp=time.time()

    else:

        if t_coly+0.15< time.time():
            sry = 0
            hy = 0
            x1, y1, w1, h1 = 0, 0, 0, 0
    cv2.rectangle(datk1, (x1, y1), (x1 + w1, y1 + h1), (0, 255,255), 2)



def cube_R():
    global xk1, yk1, xk2, yk2, lowr, upr, color_Final, perek,  srr,speed,hr,t_colr,x1, y1, w1, h1,timer_sp,cube,flag,timerCVET
    datk1 = frame[yk1:yk2, xk1:xk2]

    hsv1 = cv2.cvtColor(datk1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1, lowr, upr)
    imd1, contoursk, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    max=0

    if len(contoursk)!=0:
        timerCVET = time.time()
        for contork1 in contoursk:
            x, y, w, h = cv2.boundingRect(contork1)
            a1 = cv2.contourArea(contork1)
            if a1 > 400:
                t_colr = time.time()
                if y+h>max:
                    max=y+h
                    srr = x+w//2
                    hr = y + h
                    x1, y1, w1, h1 =  x, y, w, h
                    timer_sp=time.time()

    else:

        if t_colr+0.07< time.time():
            srr = 0
            hr = 0
            x1, y1, w1, h1 = 0, 0, 0, 0

    cv2.rectangle(datk1, (x1, y1), (x1 + w1, y1 + h1), (0, 0, 255), 2)
    cv2.rectangle(frame, (xk1, yk1), (xk2, yk2), (0, 200,200), 2)

def cube_G():
    global xk1, yk1, xk2, yk2,  lowg, upg, perek, srg,speed,hg,t_colg,x2, y2, w2, h2,timer_sp,color,xl1,xr1,hbl,hbr,timerCvet

    datg1 = frame[yk1:yk2, xk1:xk2]

    hsv1 = cv2.cvtColor(datg1, cv2.COLOR_BGR2HSV)
    maskg1 = cv2.inRange(hsv1, lowg, upg)
    imd1, contoursk, hod1 = cv2.findContours(maskg1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max = 0

    if len(contoursk) != 0:

        for contork1 in contoursk:
            x, y, w, h = cv2.boundingRect(contork1)
            a1 = cv2.contourArea(contork1)
            if a1 > 300:
                t_colg = time.time()
                if y + h > max:
                    max = y + h
                    srg = x
                    hg = y + h
                    x2, y2, w2, h2 = x, y, w, h
                    timer_sp = time.time()



    else:
        if t_colg + 0.15 < time.time():
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

def miganie(delay):
    global ts
    if ts + delay > time.time():
        LED(1, 0, 0)
    elif ts + delay*2> time.time():
        LED(1, 0, 1)
    elif ts + delay*3> time.time():
        LED(0, 0, 1)
    elif ts + delay*4 > time.time():
        LED(0, 1, 1)
    elif ts + delay*5> time.time():
        LED(0, 1, 0)
    elif ts + delay*6> time.time():
        LED(1, 1, 0)

    else:
        ts = time.time()
a=0

LED(0,0,0)

stope = 0
i=0
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
        LED(0,1,1)
        if flag_start == False:
            message = "999999$"
            miganie(0.2)
        else:
            message = "200200$"
            LED(0,0,0)

        if ii == "1":
            state = 1
            ts = time.time()
            flag_start = True


    if state == 1:  # pd

        message = str(int(deg) + 200) + str(int(speed) + 200) + '$'

        cube_Y()
        cube_G()
        cube_R()

        black_line_left()
        black_line_right()

        if srg==0 and srr==0 and sry==0:
            if speed < 95:
                speed =int(50+(time.time()-timer_sp)*50)
            pd()
            color_Final = "None"
            if t_b+0.5<time.time():
                yl1 = 180
                yr1 = 180
                hbl = 300
                hbr = 300
                yl2 = yl1 + hbl
                yr2 = yr1 + hbr
        elif srg!=0 and hy<hg>hr:
            t_b = time.time()
            if color=="orange":
                yr1 = 260
                hbr = 220
                yr2 = yr1 + hbr
            else:
                yl1 = 260
                hbl = 220
                yl2 = yl1 + hbl


            e =(215+hg*1.5)-srg
            speed=int(speed_n-hg*0.2)
            if speed < 60:
                speed = 60
            if -5 < e < 5:
                e = 0
            kp = 0.15
            kd = 2
            u = int(e * kp + (e - e_old) * kd)
            deg = 0 + u
            e_old = e

            if deg > 90:
                deg = 90
            if deg < -90:
                deg = -90
        elif hy<hr>hg and srr!=0:
            t_b=time.time()
            if color=="orange":
                yr1 = 260
                hbr = 220
                yr2 = yr1 + hbr
            else:
                yl1 = 260
                hbl = 220
                yl2 = yl1 + hbl
            e = (245-hr*1.3)-srr
            if speed<60:
                speed=60
            if -5 < e < 5:
                e = 0
            kp = 0.15
            kd = 2
            u = int(e * kp + (e - e_old) * kd)
            deg = 0 + u
            e_old = e

            if deg > 90:
                deg = 90
            if deg < -90:
                deg = -90
        elif sry!=0 and hr<hy>hg:

            if color=="blue":
                t_b = time.time()
                yl1 = 260
                hbl = 220
                yl2 = yl1 + hbl

                e = (200 + hy* 1.2) - sry
                speed = int(speed_n - hy * 0.2)
                if speed < 60:
                    speed = 60
                if -5 < e < 5:
                    e = 0
                kp = 0.15
                kd = 2
                u = int(e * kp + (e - e_old) * kd)
                deg = 0 + u
                e_old = e

                if deg > 90:
                    deg = 90
                if deg < -90:
                    deg = -90
            else:
                t_b = time.time()

                yr1 = 260
                hbr = 220
                yr2 = yr1 + hbr

                e=(270 - hy * 1.3) - sry
                if speed < 60:
                    speed = 60
                if -5 < e < 5:
                    e = 0
                kp = 0.15
                kd = 2
                u = int(e * kp + (e - e_old) * kd)
                deg = u
                e_old = e

                if deg > 90:
                    deg = 90
                if deg < -90:
                    deg = -90



        perecryostok_lyboy()

        if perek == 12 and stope == 0:
            stope = 1
            tf = time.time()

        if tf + vf < time.time() and stope == 1:
            state = 2

        if speed>100:
            speed=90
        if speed < 0:
            speed = 0






    if state == 2:  # stop
        message = str(int(0) + 200) + str(int(0) + 200) + '$'
        # for i in range(len(list)):
        #     text+=list[i]
        #     text+=", "

        # robot.text_to_frame(frame, 'list = ' + str(text), 3, 140)



    if state == 5:  # rulevoe
        miganie(0.5)
        if key == 87:  # это W
            speed += 3

        elif key == 83:  # это S
            speed -= 3

        elif key == 65:  # это A
            deg += 3

        elif key == 68:  # это D
            deg -= 3



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

    robot.text_to_frame(frame, 'fps = ' + str(fps), 3, 20)
    robot.text_to_frame(frame, 'deg = ' + str(deg), 3, 40)
    robot.text_to_frame(frame, 'speed = ' + str(speed), 250, 40)
    robot.text_to_frame(frame, 'message = ' + str(message), 3, 60)
    robot.text_to_frame(frame, 'sr1-sr2= ' + str(sr1)+'-' +str(sr2) + ' color = ' + str(color), 3, 80)
    robot.text_to_frame(frame,  'srr ,srg= ' + str(srr) + ' ,' + str(srg) + 'hr ,hg= ' + str(hr) + ' ,' + str(hg),3,100)
    robot.text_to_frame(frame, 'perekryostok ' + str(perek), 3, 120)







    robot.set_frame(frame, 40)

    # if key > -1:
    #     print(str(key) + " state "+str(state))