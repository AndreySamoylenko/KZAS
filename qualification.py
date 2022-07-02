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
R=11
B=15
G=13

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

# ниже идёт различное HSV для поиска цветов
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


xp1, yp1 = 280, 400 # координаты для расположения датчиков
hp, wp =40,80
xp2,yp2 = xp1+wp,yp1+hp
# d1 = frame[220:260, 0:100]
# d2 = frame[260:300, 0:200]
# d3 = frame[220:260, 540:640]
# d4 = frame[260:300, 440:640]
x11,y11=0,220
x12,y12=0,260
w1,h1=125,40
w2=250
x13,y13=640-w1,y11
x14,y14=640-w2,y12
x21,y21=x11+w1,y11+h1
x22,y22=x12+w2,y12+h1
x23,y23=x13+w1,y13+h1
x24,y24=x14+w2,y14+h1


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
vf=0

state=0         # переменные состояния
stope = 0

t111=time.time()    # таймеры
tf=time.time()
ts=time.time()
tim_per=time.time()

ii = "" # для uart

flag_start=False  # флаги
flag_l=False

vrem_list=[0,0,0,0]   # список времени зон для функции x_road()

def __(d1):                                                         # функция поиска чёрных объектов(бортиков)
    dat1 = cv2.GaussianBlur(d1, (5, 5), cv2.BORDER_DEFAULT)                                 # эти 4 строчки используются
    gr1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2GRAY)                                            # для поиска контуров на
    _, maskd1 = cv2.threshold(gr1, 60, 255, cv2.THRESH_BINARY_INV)                          # чёрно-белой картинке
    imd1, contoursd1, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #
    max1 = 0
    pok1 = 0
    for contorb1 in contoursd1:                                     # этот цикл ищет наибольший контур поплощади и
                                                                    # высчитывает среднее значение
        x, y, w, h = cv2.boundingRect(contorb1)
        a1 = cv2.contourArea(contorb1)
        if a1 > 500:
            if max1 < h * w:
                pok1 = h * (x + w)
                max1 = h * w
            # cv2.rectangle(d1, (x + 1, y + 1), (x + w - 1, y + h - 1), (10, 245, 0), 2) это должно обрисовывать
            # кортуры ,но это конфликтовало с поиском кубиков
    return pok1

def ___(d1,w1):# эта функция идентична предыдущей , но использует другую формулу для подсчёта среднего значения
    dat1 = cv2.GaussianBlur(d1, (5, 5), cv2.BORDER_DEFAULT)
    gr1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2GRAY)
    _, maskd1 = cv2.threshold(gr1, 60, 255, cv2.THRESH_BINARY_INV)
    imd1, contoursd1, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max1 = 0
    pok1 = 0
    for contorb1 in contoursd1:
        x, y, w, h = cv2.boundingRect(contorb1)
        a1 = cv2.contourArea(contorb1)
        if a1 > 500:
            if max1 < h * w:
                pok1 = h * (w1-x)
                max1 = h * w
            # cv2.rectangle(d1, (x + 1, y + 1), (x + w - 1, y + h - 1), (10, 245, 0), 2)
    return pok1

def pd(sr1,sr2,kp,kd):              # пропорционально-дифференциальный регулятор
    global e,e_old,deg,color        #
                                    #
                                    #
    e = sr2 - sr1                   #
    if -5 < e < 5:
        e = 0
    u = int(e * kp + (e - e_old) * kd)
    deg =  u
    e_old = e                       # до сюда обычный пропорциональный регулятор

    if sr1 == 0:
        deg = 55
    if sr2 == 0:
        deg = -55
    if sr1==0 and sr2==0:
        if color == "orange":
            deg = -30
        else:
            deg =30
    if deg > 90:
        deg = 90
    if deg < -90:
        deg = -90                   # различные полезные фишки

def x_road():# функция поиска перекрёстков
    global xp1, yp1, xp2, yp2, lowb, upb, lowo, upo, color, perek, t111,flag_l,tim_per,vf,i
    datp1 = frame[yp1:yp2, xp1:xp2]
    cv2.rectangle(datp1,(xp1,yp1),(xp2,yp2),(200,100,100),3)
    if color==None or color=="blue":

        hsv1 = cv2.cvtColor(datp1.copy(), cv2.COLOR_BGR2HSV)
        maskd1 = cv2.inRange(hsv1, lowb, upb)    #
        imd1, contoursb, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)# поиск синего перекрёстка


        for contorb1 in contoursb:
            x, y, w, h = cv2.boundingRect(contorb1)
            a1 = cv2.contourArea(contorb1)
            if a1 > 500  and  t111 + 0.9 < time.time():
                if perek<5:                             # подсчёт времени для каждого участка трассы между перекрёстками
                    vrem_list[perek%4]=round(time.time() - tim_per,2)
                    tim_per = time.time()
                else:
                    vf = vrem_list[0]* 0.6
                color = "blue"
                LED(0,0,1)
                flag_l=True
                # cv2.drawContours(datp1,contoursb,0,(0,0,0),3)
                cv2.rectangle(datp1, (x, y), (x + w, y + h), (255, 0, 0), 2)     # подсчёт перекрёстков
                perek+=1
                i=0
                t111=time.time()


    if color == None or color == "orange":
        hsv2 = cv2.cvtColor(datp1.copy(), cv2.COLOR_BGR2HSV)                # всё тоже самое только для оранжевого
        maskd2 = cv2.inRange(hsv2, lowo, upo)  #
        imd1, contourso, hod1 = cv2.findContours(maskd2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        for contoro in contourso:
            x, y, w, h = cv2.boundingRect(contoro)
            a1 = cv2.contourArea(contoro)
            if a1 > 500 and  t111 + 0.6< time.time():
                if perek < 5:
                    vrem_list[perek % 4] = round(time.time() - tim_per, 2)
                    tim_per = time.time()
                else:
                    vf=vrem_list[0]* 0.6
                color = "orange"
                LED(1, 1, 0)
                perek += 1
                i=0
                flag_l=True

                t111 = time.time()
                # cv2.drawContours(datp1, contourso, 0, (100, 100, 100), 3)
                cv2.rectangle(datp1, (x, y), (x + w, y + h), (0, 100, 255), 2)
    cv2.rectangle(frame, (xp1, yp1), (xp2, yp2), (0, 205, 0), 2)

def black_line():
    global x11, x12, x13, x14, x21, x22, x23, x24, y11, y12, y13, y14, y21, y22, y23, y24, w11, w12, h11, sr1, sr2
    d1 = frame[y11:y21, x11:x21]
    d2 = frame[y12:y22, x12:x22]
    d3 = frame[y13:y23, x13:x23]
    d4 = frame[y14:y24, x14:x24]

    sr1 = (__(d1)+__(d2)) // 100
    sr2 = (___(d3,125)+___(d4,250)) // 100

    # cv2.rectangle(frame, (x11, y11), (x21, y21), (0, 0, 10), 2)
    # cv2.rectangle(frame, (x12, y12), (x22, y22), (0, 0, 10), 2)
    # cv2.rectangle(frame, (x13, y13), (x23, y23), (0, 0, 10), 2)
    # cv2.rectangle(frame, (x14, y14), (x24, y24), (0, 0, 10), 2)

def LED(r,g,b):
    if r==0:        # упрощённое управление RGB светодиодом
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

def miganie(delay):
    global ts       # относительно бесполезная функция для свечения RGB светодиодом
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

LED(0,0,0)


while 1:
    key = robot.get_key()
    if flag_l==True and t111+0.5<time.time() and state==1:  # выключение светодиода после перекрёстка
        LED(0,0,0)
        flag_l=False

    if key != -1:    # обработка клавиш для смены состояний
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

    if state == 0: # ожидание кнопки
        if flag_start == False:
            message = "999999$"
            miganie(0.2)
        else:
            message = "200200$"
            LED(0, 0, 0)

        if ii == "1":
            state = 1
            ts = time.time()
            flag_start = True


    if state==1:# езда и счёт перекрёстков
        black_line()
        pd(sr1,sr2,0.3,3)
        x_road()
        if perek == 12 and stope == 0: # логика для финиша
            stope = 1
            tf = time.time()

        if tf + vf< time.time() and stope == 1:# финиш
            state = 2

        message = str(int(deg) + 200) + str(int(speed) + 200) + '$'# формируем сообщение

    if state == 2:  # stop
        message ='200200$'# отправляем 0 0


        message = str(int(0) + 200) + str(int(0) + 200) + '$'


    # if state==5:# ручное управление
    #
    #     if key==87:# это W
    #         speed+=3
    #
    #     elif key==83:# это S
    #         speed-=3
    #
    #     elif key==65:# это A
    #         deg+=3
    #
    #     elif key==68:# это D
    #         deg-=3
    #
    #     elif key==32:
    #         speed=0
    #         deg=0
    #     elif key==81:
    #         deg=0
    #     if speed > 100:
    #         speed =100
    #     if speed < -100:
    #         speed = -100
    #     if deg>70:
    #         deg=60
    #     if deg <  -70:
    #         deg = -60
    #
    #     message = str(int(deg) + 200) + str(int(speed) + 200) + '$'




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
    robot.text_to_frame(frame, 'message = ' + str(message) +  ' ii = ' + str(ii), 50, 60)
    robot.text_to_frame(frame, 'perek = ' + str(perek)  +  ' color = ' + str(color), 50, 80)
    robot.text_to_frame(frame, 'sr1 - sr2 = ' + str(sr1) + ' - ' + str(sr2) + ' = ' + str(e), 50, 100)
    robot.set_frame(frame, 40)

    # if key > -1:
    #     print(str(key) + " state "+str(state))