import cv2
import RobotAPI as rapi
import numpy as np
import serial
import time

port = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)
robot = rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)

fps = 0
fps1 = 0
fps_time = 0
message = ""

# ниже идёт различное HSV для поиска цветов
#[39, 70 ,84] [111, 255, 164] blue
#[ 0, 87,87] [ 66 ,186 ,166] orange

lowblack = np.array([35 ,67 , 0] )
upblack = np.array([103 ,256 , 29])

lowblue = np.array([ 91 ,135,   6])
upblue = np.array([117, 255, 164])

loworange = np.array([  3 , 79 ,113])
uporange = np.array([ 59 ,210, 189])

lowr = np.array([ 0, 89, 47])
upr = np.array([  6,223 ,165])

lowg = np.array([ 64, 200 , 56])
upg = np.array([ 81, 255 ,210])



#     d1 = frame[220:260, 0:100]
#     d2 = frame[260:300, 0:200]
#     d3 = frame[220:260, 540:640]
#     d4 = frame[260:300, 440:640]

x_line_dat = [0, 100, 0, 200, 540, 640, 440, 640]
y_line_dat = [220, 260, 260, 300, 220, 260, 260, 300]
x_perek = [280,360]
y_perek = [330,370]

e_old = 0                           #различные переменные для ПД
sr1 = 0
sr2 = 0
speed = 3
perek=0
color_per=None
kp = 0.5
kd = 5
deg=0
u=0
e=0
dat1,dat2=0,0

state=0                             # переменные состояния
stope = 0

x_road_tim=time.time()                    # таймеры
tf=time.time()
ts=time.time()

ii = ""                             # для uart

flag_start=False                    # флаги
flag_l=False

vrem_list=[]    # список времени зон для функции x_road()

def wait_for_key():
    message = str('999999$')
    ii='0'
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

def black_poisk_l(d1):
    xm, ym, wm, hm =0,0,0,0
    dat=cv2.GaussianBlur(d1,(5,5),cv2.BORDER_DEFAULT)
    hsv = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)
    blur = cv2.blur(hsv,(5,5))
    mask = cv2.inRange(blur, lowblack, upblack)  #

    imd1, contours, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  #
    max1 = 0
    dat = 0
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area > 500:
            if max1 < h * w:
                max1 = h * w
                dat = h * (w+x)
                xm, ym, wm, hm = x, y, w, h

    return [xm + 1,ym + 1,xm + wm - 1, ym + hm - 1,dat]

def black_poisk_r(d1,w_dat):
    xm, ym, wm, hm =0,0,0,0
    dat=cv2.GaussianBlur(d1,(5,5),cv2.BORDER_DEFAULT)
    hsv = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)
    blur = cv2.blur(hsv,(5,5))
    mask = cv2.inRange(blur, lowblack, upblack)  #

    im, contours, ho = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  #
    max1 = 0
    dat = 0
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area > 500:
            if max1 < h * w:
                max1 = h * w
                dat = h * (w_dat-x)
                xm, ym, wm, hm = x, y, w, h


    return [xm + 1,ym + 1,xm + wm - 1, ym + hm - 1,dat]

def bortik_pro():
    global x_line_dat, y_line_dat, dat1, dat2

    d1 = frame[y_line_dat[0]:y_line_dat[1], x_line_dat[0]:x_line_dat[1]]
    d2 = frame[y_line_dat[2]:y_line_dat[3], x_line_dat[2]:x_line_dat[3]]
    d3 = frame[y_line_dat[4]:y_line_dat[5], x_line_dat[4]:x_line_dat[5]]
    d4 = frame[y_line_dat[6]:y_line_dat[7], x_line_dat[6]:x_line_dat[7]]  # забираем часть экрана длядатчиков

    dat1 = (black_poisk_l(d1)[4] + black_poisk_l(d2)[4]) // 100  # высчитываем среднее исходя из показаний датчиков
    dat2 = (black_poisk_r(d3, 125)[4] + black_poisk_r(d4, 250)[4]) // 100

def pd_regulator(dat1,dat2):              # пропорционально-дифференциальный регулятор
    global e, e_old, deg, color_per, u

    e = dat2 - dat1
    if -5 < e < 5:
        e = 0
    u = int(e * kp + (e - e_old) * kd)
    deg = u
    e_old = e                       # до сюда обычный пропорционально-дифференциальный регулятор

    if dat1 == 0:
        deg = 25
    if dat2 == 0:
        deg = -25

    if dat1 == 0 and dat2 == 0:
        if color_per == "orange":
            deg = -30
        elif color_per == "blue":
            deg = 30

    if deg > 90:
        deg = 90
    if deg < -90:
        deg = -90                   # различные полезные фишки

def x_road():# функция поиска перекрёстков
    global xp1, yp1, xp2, yp2, lowb, upb, lowo, upo, color, perek, x_road_tim, flag_l, tim_per, vf, i
    datp1 = frame[yp1:yp2, xp1:xp2]
    cv2.rectangle(datp1,(xp1,yp1),(xp2,yp2),(200,100,100),3)
    if color==None or color=="blue":

        hsv1 = cv2.cvtColor(datp1.copy(), cv2.COLOR_BGR2HSV)
        maskd1 = cv2.inRange(hsv1, lowb, upb)    #
        imd1, contoursb, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)# поиск синего перекрёстка


        for contorb1 in contoursb:
            x, y, w, h = cv2.boundingRect(contorb1)
            a1 = cv2.contourArea(contorb1)
            if a1 > 500  and  x_road_tim + 0.9 < time.time():
                if perek<5:                             # подсчёт времени для каждого участка трассы между перекрёстками
                    vrem_list[perek%4]=round(time.time() - tim_per,2)
                    tim_per = time.time()
                else:
                    vf = vrem_list[0]* 0.6
                color = "blue"
                flag_l=True
                # cv2.drawContours(datp1,contoursb,0,(0,0,0),3)
                cv2.rectangle(datp1, (x, y), (x + w, y + h), (255, 0, 0), 2)     # подсчёт перекрёстков
                perek+=1
                i=0
                x_road_tim=time.time()


    if color == None or color == "orange":
        hsv2 = cv2.cvtColor(datp1.copy(), cv2.COLOR_BGR2HSV)                # всё тоже самое только для оранжевого
        maskd2 = cv2.inRange(hsv2, lowo, upo)  #
        imd1, contourso, hod1 = cv2.findContours(maskd2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        for contoro in contourso:
            x, y, w, h = cv2.boundingRect(contoro)
            a1 = cv2.contourArea(contoro)
            if a1 > 500 and x_road_tim + 0.6< time.time():
                if perek < 5:
                    vrem_list[perek % 4] = round(time.time() - tim_per, 2)
                    tim_per = time.time()
                else:
                    vf=vrem_list[0]* 0.6
                color = "orange"
                perek += 1
                i=0
                flag_l=True

                x_road_tim = time.time()
                # cv2.drawContours(datp1, contourso, 0, (100, 100, 100), 3)
                cv2.rectangle(datp1, (x, y), (x + w, y + h), (0, 100, 255), 2)
    cv2.rectangle(frame, (xp1, yp1), (xp2, yp2), (0, 205, 0), 2)





wait_for_key()

while 1:
    key = robot.get_key()
    frame = robot.get_frame(wait_new_frame=1)

    l1 = black_poisk_l(frame[y_line_dat[0]:y_line_dat[1], x_line_dat[0]:x_line_dat[1]])
    l2 = black_poisk_l(frame[y_line_dat[2]:y_line_dat[3], x_line_dat[2]:x_line_dat[3]])
    l3 = black_poisk_r(frame[y_line_dat[4]:y_line_dat[5], x_line_dat[4]:x_line_dat[5]], 100)
    l4 = black_poisk_r(frame[y_line_dat[6]:y_line_dat[7], x_line_dat[6]:x_line_dat[7]], 200)

    cv2.rectangle(frame, (x_line_dat[0], y_line_dat[0]), (x_line_dat[1], y_line_dat[1]), (0, 25, 200), 2)
    cv2.rectangle(frame, (x_line_dat[2], y_line_dat[2]), (x_line_dat[3], y_line_dat[3]), (0, 25, 200), 2)
    cv2.rectangle(frame, (x_line_dat[4], y_line_dat[4]), (x_line_dat[5], y_line_dat[5]), (0, 25, 200), 2)
    cv2.rectangle(frame, (x_line_dat[6], y_line_dat[6]), (x_line_dat[7], y_line_dat[7]), (0, 25, 200), 2)

    cv2.rectangle(frame, (x_perek[0], y_perek[0]), (x_perek[1], y_perek[1]), (0, 205, 200), 2)

    cv2.rectangle(frame[y_line_dat[0]:y_line_dat[1], x_line_dat[0]:x_line_dat[1]], (l1[0], l1[1]), (l1[2], l1[3]),(10, 245, 0), 2)
    cv2.rectangle(frame[y_line_dat[2]:y_line_dat[3], x_line_dat[2]:x_line_dat[3]], (l2[0], l2[1]), (l2[2], l2[3]),(10, 245, 0), 2)
    cv2.rectangle(frame[y_line_dat[4]:y_line_dat[5], x_line_dat[4]:x_line_dat[5]], (l3[0], l3[1]), (l3[2], l3[3]),(10, 245, 0), 2)
    cv2.rectangle(frame[y_line_dat[6]:y_line_dat[7], x_line_dat[6]:x_line_dat[7]], (l4[0], l4[1]), (l4[2], l4[3]),(10, 245, 0), 2)

    bortik_pro()

    pd_regulator(dat1, dat2)

    #x_road()

    fps1 += 1
    if time.time() > fps_time + 1:
        fps_time = time.time()
        fps = fps1
        fps1 = 0

    message = str(int(deg) + 200) + str(int(speed) + 200) + '$'
    port.write(message.encode("utf-8"))

    robot.text_to_frame(frame, 'fps = ' + str(fps), 50, 20)
    robot.text_to_frame(frame, dat1, 0, 140)
    robot.text_to_frame(frame, dat2, 600, 140)
    robot.text_to_frame(frame, deg, 300, 200)
    robot.set_frame(frame, 40)
