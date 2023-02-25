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

lowo = np.array([  3 , 79 ,113])
upo = np.array([ 59 ,210, 189])

lowr = np.array([ 0, 89, 47])
upr = np.array([  6,223 ,165])

lowg = np.array([ 64, 200 , 56])
upg = np.array([ 81, 255 ,210])


# d1 = frame[120:480, 0:60]
# d2 = frame[120:480, 580:640]
x=[0,60,580,640]
y=[160,480,160,480]

e_old = 0 #различные переменные для ПД
sr1 = 0
sr2 = 0
speed = 90
perek=0
color=None
kp = 0.7
kd = 5
deg=0
u=0
e=0

state=0         # переменные состояния
stope = 0

t111=time.time()    # таймеры
tf=time.time()
ts=time.time()

ii = "" # для uart

flag_start=False  # флаги
flag_l=False

vrem_list=[]    # список времени зон для функции x_road()

def nigga_poisk(d1):
    xm, ym, wm, hm =0,0,0,0
    dat=cv2.GaussianBlur(d1,(5,5),cv2.BORDER_DEFAULT)
    hsv = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)
    blur = cv2.blur(hsv,(5,5))
    mask = cv2.inRange(blur, lowblack, upblack)  #

    imd1, contours, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  #
    max1 = 0
    pok1 = 0
    for contour in contours:  # этот цикл ищет наибольший контур поплощади и
        # высчитывает среднее значение
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area > 500:
            if max1 < h * w:
                max1 = h * w
                xm, ym, wm, hm = x, y, w, h
            # кортуры ,но это конфликтовало с поиском кубиков
    return [xm + 1,ym + 1,xm + wm - 1, ym + hm - 1]

while 1:
    key = robot.get_key()
    frame = robot.get_frame(wait_new_frame=1)

    l1 = nigga_poisk(frame[y[0]:y[1], x[0]:x[1]])
    cv2.rectangle(frame[y[0]:y[1], x[0]:x[1]],(l1[0],l1[1]),(l1[2],l1[3]), (10, 245, 0), 2)
    cv2.rectangle(frame, (x[0], y[0]), (x[1], y[1]), (0, 25,200), 2)

    l2 = nigga_poisk(frame[y[2]:y[3], x[2]:x[3]])
    cv2.rectangle(frame[y[2]:y[3], x[2]:x[3]], (l2[0], l2[1]), (l2[2], l2[3]), (10, 245, 0), 2)
    cv2.rectangle(frame, (x[2], y[2]), (x[3], y[3]), (0, 25, 200), 2)

    fps1 += 1
    if time.time() > fps_time + 1:
        fps_time = time.time()
        fps = fps1
        fps1 = 0

    robot.text_to_frame(frame, 'fps = ' + str(fps), 50, 20)
    robot.set_frame(frame, 40)
