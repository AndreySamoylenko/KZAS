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
#[164  , 0  , 0] [180 ,255 ,255]
#[ 72, 121 , 48] [ 93 ,255 ,211]green
#[ 28, 176 , 74] [ 52, 255, 236] yellow

lowb = np.array([ 91 ,135,   6])
upb = np.array([117, 255, 164])

lowo = np.array([ 8, 78 ,86])
upo = np.array( [ 30, 218 ,235])

lowr = np.array([  0 ,106,  66])
upr = np.array([  4,255,255])

lowr1 = np.array([164  , 0  , 0])
upr1 = np.array([180 ,255 ,255])

lowg = np.array([ 64, 200 , 56])
upg = np.array([ 81, 255 ,210])

lowy= np.array([ 30, 176 , 74])
upy=np.array([ 52, 255, 236])

xl1, yl1 = 0, 200
xr1, yr1 = 620, 200
xp1, yp1 = 300, 400
xk1, yk1 = 35,200

hbl, wbl= 280, 20
hbr,wbr=280,20
hp, wp = 25, 40
hk, wk = 180, 580


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
dop_per=0

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
timer_z=time.time()
vremf=0
vf=1.9

ii = ""
flag_start = False
flag_l=False
col_g=False
flag_g=True
col_r=False
flag_r=True
col_y=False
flag_y=True

list=[[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
List=[[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
vrem_list=[0,0,0,0]
abc=[]

hg=0
hr=0
hy=0
x1, y1, w1, h1 =0,0,0,0
x2, y2, w2, h2 =0,0,0,0

# for i in range(100):
#     for j in range(2):
#         list[i][j]=None




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
    d2 = frame[y12:y22, x12:x22]# забираем часть экрана длядатчиков
    d3 = frame[y13:y23, x13:x23]
    d4 = frame[y14:y24, x14:x24]

    sr1 = (__(d1)+__(d2)) // 100            # высчитываем среднее исходя из показаний датчиков
    sr2 = (___(d3,125)+___(d4,250)) // 100

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

def cube_Y():# функция поиска жёлтых кубиков
    global xk1, yk1, xk2, yk2, lowy, upy, color_Final, perek,  sry,speed,hy,t_coly,timer_sp,cube,flag,\
        timerCVET,dop_per,cub
    datk1 = frame[yk1:yk2, xk1:xk2]
    x1, y1, w1, h1 = 0, 0, 0, 0
    hsv1 = cv2.cvtColor(datk1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1, lowy, upy)
    maskd2 =cv2.inRange(hsv1  ,lowr,upr)
    mask2=cv2.bitwise_not(maskd2)
    mask=cv2.bitwise_and(mask2,maskd1)
    imd1, contoursk, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

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

        if t_coly+0.09< time.time():
            sry = 0
            hy = 0
            x1, y1, w1, h1 = 0, 0, 0, 0



    cv2.rectangle(datk1, (x1, y1), (x1 + w1, y1 + h1), (0, 255,255), 2)

def cube_R():# функция поиска красных кубиков
    global xk1, yk1, xk2, yk2, lowr, upr, color_Final, perek, i, srr,speed,hr,t_colr,timer_sp,cube,flag,timerCVET,\
        flag_wr,tim_per
    datk1 = frame[yk1:yk2, xk1:xk2]

    hsv1 = cv2.cvtColor(datk1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1, lowr, upr)
    maskd2=cv2.inRange(hsv1,lowr1,upr1)             # поиск по HSV
    mask=cv2.bitwise_or(maskd1,maskd2)
    imd1, contoursk, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    x1, y1, w1, h1=0,0,0,0
    max=0

    if len(contoursk)!=0:
        timerCVET = time.time()
        for contork1 in contoursk:
            x, y, w, h = cv2.boundingRect(contork1)
            a1 = cv2.contourArea(contork1)
            if a1 >350:
                t_colr = time.time()
                LED(1,0,0)
                if y+h>115 and flag_wr and perek<5:# запись кубиков в список
                    list[perek%4][i]=5
                    flag_wr=False
                    List[perek%4][i]=round(time.time()-tim_per,1)
                    i=2
                if y+h<95 and not flag_wr:
                    flag_wr=True


                if y+h>max:      # поиск наибольшего
                    max=y+h
                    srr = x+w
                    hr = y + h
                    x1, y1, w1, h1 =  x, y, w, h
                    timer_sp=time.time()
            else:
                if t_colr + 0.05 < time.time():# таймер для исчезновения кубика
                    srr = 0
                    hr = 0
                    x1, y1, w1, h1 = 0, 0, 0, 0
    else:

        if t_colr+0.05< time.time():
            srr = 0
            hr = 0
            x1, y1, w1, h1 = 0, 0, 0, 0

    cv2.rectangle(datk1, (x1, y1), (x1 + w1, y1 + h1), (0, 0, 255), 2)
    cv2.rectangle(frame, (xk1, yk1), (xk2, yk2), (0, 200,200), 2)

def cube_G():# функция поиска зеленых кубиков (за подробностями в cube_R())
    global xk1, yk1, xk2, yk2,  lowg, upg, perek, srg,speed,hg,t_colg,timer_sp,color,xl1,xr1,hbl,hbr,timerCvet,flag_wg,i

    datg1 = frame[yk1:yk2, xk1:xk2]
    x2, y2, w2, h2 = 0, 0, 0, 0
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
                LED(0,1,0)
                if y + h > 115  and flag_wg and perek < 5:
                    list[perek % 4][i] = 3
                    flag_wg = False
                    List[perek % 4][i] = round(time.time() - tim_per, 1)
                    i = 2
                if y + h < 115  and not flag_wg:
                    flag_wg = True

                if y + h > max:
                    max = y + h
                    srg = x
                    hg = y + h
                    x2, y2, w2, h2 = x, y, w, h
                    timer_sp = time.time()
            else:
                if t_colg + 0.07 < time.time():
                    srg = 0
                    hg = 0
                    x2, y2, w2, h2 = 0, 0, 0, 0



    else:
        if t_colg + 0.05  < time.time():
            srg = 0
            hg = 0
            x2, y2, w2, h2 = 0, 0, 0, 0


    cv2.rectangle(datg1, (x2, y2), (x2 + w2, y2 + h2), (0, 255, 0), 2)


LED(0,0,0)
b=0
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

    if state == 0:  # ожидание кнопки
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


    if state == 1:  # езда

        message = str(int(deg) + 200) + str(int(speed) + 200) + '$'

        cube_Y()    # поиск кубов
        cube_G()
        cube_R()

        black_line()

        if srg==0 and srr==0 and sry==0:    # пропорционально-дифференциальный регулятор

            if t_b + 55 / speed > time.time():  # после кубика поворот в направлении движения
                if color == "orange" and color_Final == "Green":
                    sr2 = 0
                elif color_Final == "Red" and color == "blue":
                    sr1 = 0
                b = 1
            elif b == 1:
                color_Final = "None"
                LED(1, 1, 1)
                b = 0
            pd(sr1, sr2, 0.3, 3)

        elif srg!=0 and hy<hg>hr: # если есть зелёный куб и он ближе
            LED(0,1,0)
            color_Final = "Green"
            t_b = time.time()
            e =(210+hg*1.3)-srg # ошибка высчитывается исходя из перспективы
            if -5 < e < 5:
                e = 0
            kp =0.2
            kd =0.2
            u = int(e * kp + (e - e_old) * kd)      # пропорционально-дифференциальный регулятор
            deg = 0 + u
            e_old = e

            if deg > 90:
                deg = 90
            if deg < -90:
                deg = -90
        elif hy<hr>hg and srr!=0: # если есть красный и он ближе
            LED(1,0,0)
            color_Final = "Red"
            t_b=time.time()
            e = (270-hr*1.3)-srr

            if -5 < e < 5:
                e = 0
            kp = 0.2
            kd = 0.2
            u = int(e * kp + (e - e_old) * kd)
            deg = 0 + u
            e_old = e

            if deg > 90:
                deg = 90
            if deg < -90:
                deg = -90

        elif sry!=0 and hr<hy>hg: # тут добавлен поиск жёлтого куба
            #if cub==0:
            #    cub=1
            #    dop_per=perek
            if color=="orange":

                LED(0, 0, 1)

                t_b = time.time()
                e = (270 - hr * 1.3) - srr

                if -5 < e < 5:
                    e = 0
                kp = 0.2
                kd = 0.2
                u = int(e * kp + (e - e_old) * kd)
                deg = 0 + u
                e_old = e

                if deg > 90:
                    deg = 90
                if deg < -90:
                    deg = -90

            else:
                LED(1, 1, 0)

                t_b = time.time()
                e = (210 + hg * 1.3) - srg  # ошибка высчитывается исходя из перспективы
                if -5 < e < 5:
                    e = 0
                kp = 0.2
                kd = 0.2
                u = int(e * kp + (e - e_old) * kd)  # пропорционально-дифференциальный регулятор
                deg = 0 + u
                e_old = e

                if deg > 90:
                    deg = 90
                if deg < -90:
                    deg = -90




        x_road()
                    # + dop_per
        if perek == 12 and stope == 0:  # переход в финиш
            # state=3
            # ts=time.time()
            stope = 1
            tf = time.time()

        if tf + vf < time.time() and stope == 1:    # финиш
            state = 2

        if speed>100:   # ограничение по скорости
            speed=90
        if speed < 0:
            speed = 0






    if state == 2:  # stop
        message = str(int(0) + 200) + str(int(0) + 200) + '$'
        # for i in range(len(list)):
        #     text+=list[i]
        #     text+=", "

        # robot.text_to_frame(frame, 'list = ' + str(text), 3, 140)

    if state==3:    # Разворот
        if ts+0.3>time.time():
            if color=="orange":
                deg=-60
            else:
                deg=60
            speed = 60
        elif ts +1.2>time.time():
            if color=="blue":
                deg=-60
            else:
                deg=60
            speed=60
        else:
            state=1
        message = str(int(deg) + 200) + str(int(speed) + 200) + '$'



    # if state == 5:  # rulevoe
    #     miganie(0.5)
    #     if key == 87:  # это W
    #         speed += 3
    #
    #     elif key == 83:  # это S
    #         speed -= 3
    #
    #     elif key == 65:  # это A
    #         deg += 3
    #
    #     elif key == 68:  # это D
    #         deg -= 3
    #
    #
    #
    #     elif key == 32:
    #         speed = 0
    #         deg = 0
    #     if speed > 80:
    #         speed = 80
    #     if speed < -80:
    #         speed = -80
    #     if deg > 70:
    #         deg = 60
    #     if deg < - 70:
    #         deg = -60
    #
    #     message = str(int(deg) + 200) + str(int(speed) + 200) + '$'

    fps1 += 1       # отсчёт FPS
    if time.time() > fps_time + 1:
        fps_time = time.time()
        fps = fps1
        fps1 = 0

    port.write(message.encode("utf-8")) # отправка сообщения
    if port.in_waiting > 0:             # приём сообщения (кнопки)
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

    robot.text_to_frame(frame, 'fps = ' + str(fps), 3, 20)      # телеметрия
    robot.text_to_frame(frame, 'state= ' + str(state), 3, 40)
    robot.text_to_frame(frame, 'deg = ' + str(deg), 265, 475)
    robot.text_to_frame(frame, 'speed = ' + str(speed), 250 , 455 )
    robot.text_to_frame(frame, 'message = ' + str(message), 3, 60)
    robot.text_to_frame(frame, 'sr1-sr2= ' + str(sr1)+'-' +str(sr2) + ' color = ' + str(color), 3, 80)
    robot.text_to_frame(frame,  'srr , srg , sry = ' + str(srr) + ' , ' + str(srg) +  str(sry) + ' , '
                        +'hr , ''hg, hy= ' + str(hr) + ' ,' + str(hg)+ ' ,' + str(hy),3,100)
    robot.text_to_frame(frame, 'perekryostok ' + str(perek), 3, 120)
    robot.text_to_frame(frame,"dop_per= "+str(dop_per),440,60,(200,255,250))

    robot.text_to_frame(frame, 'vf= ' + str(vremf), 460, 20)








    robot.set_frame(frame, 40)

    # if key > -1:
    #     print(str(key) + " state "+str(state))