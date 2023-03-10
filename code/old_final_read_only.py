import cv2
import RobotAPI as rapi
import numpy as np
import serial
import time
import RPi.GPIO as GPIO

port = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)
robot = rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)
i = 0
# LED
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
R = 11
B = 15
G = 13

GPIO.setup(R, GPIO.OUT)
GPIO.setup(G, GPIO.OUT)
GPIO.setup(B, GPIO.OUT)

GPIO.output(R, GPIO.HIGH)
GPIO.output(G, GPIO.HIGH)
GPIO.output(B, GPIO.HIGH)

message = ""
fps = 0
fps1 = 0
fps_time = 0

b = 0
# [ 95 ,151 , 28] [135, 255 , 95] blue
# [  4 ,104,  99] [ 17, 202 ,255] orange
# [0 ,0, 0] [  6, 255, 255] red
# [ 58, 132,  71] [ 74, 255, 223]green     ниже идёт HSV


lowb = np.array([91, 135, 6])
upb = np.array([117, 255, 164])

lowo = np.array([3, 50, 70])
upo = np.array([59, 210, 189])

lowr = np.array([0, 0, 0])
upr = np.array([6, 245, 243])

lowg = np.array([58, 132, 71])
upg = np.array([74, 255, 223])

lowr1 = np.array([164, 0, 0])
upr1 = np.array([180, 255, 255])

# коодинаты для датчиков
xk1, yk1 = 80, 180
hk, wk = 190, 480
xk2, yk2 = xk1 + wk, yk1 + hk

xp1, yp1 = 280, 400
hp, wp = 40, 80
xp2, yp2 = xp1 + wp, yp1 + hp

# d1 = frame[220:260, 0:100]
# d2 = frame[260:300, 0:200]
# d3 = frame[220:260, 540:640]
# d4 = frame[260:300, 440:640]

w11, h11 = 80, 30
w12 = 250

x11, y11 = 0, 195
x12, y12 = 0, y11 + h11

x13, y13 = 640 - w11, y11
x14, y14 = 640 - w12, y12

x21, y21 = x11 + w11, y11 + h11
x22, y22 = x12 + w12, y12 + h11
x23, y23 = x13 + w11, y13 + h11
x24, y24 = x14 + w12, y14 + h11

sr1 = 0  # переменные для езды
sr2 = 0
srg = 0
srr = 0
hg = 0
hr = 0
aa = 0
speed = 80
perek = 0
color = None
color_Final = ""
e_old = 0
stope = 0
state = 0
func = None
kp = 0.7
kd = 5
deg = 0
u = 0
e = 0

# таймеры
timer_per = time.time()
tf = time.time()
ts = time.time()
t_colr = time.time()
t_colg = time.time()
tim_per = time.time()
t_b = time.time()
timer_finish = time.time()
vf = 1.7
t111 = time.time()

ii = ""

# флаги
flag_start = False
flag_l = False
pyb_flag = False
col_g = False
flag_g = True
col_r = False
flag_r = True
flag_wr = True
flag_wg = True
fff = False

# списки
list = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
List = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
vrem_list = [0, 0, 0, 0]
abc = []


def __(d1):  # функция поиска чёрных объектов(бортиков)
    dat1 = cv2.GaussianBlur(d1, (5, 5), cv2.BORDER_DEFAULT)  # эти 4 строчки используются
    gr1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2GRAY)  # для поиска контуров на
    _, maskd1 = cv2.threshold(gr1, 60, 255, cv2.THRESH_BINARY_INV)  # чёрно-белой картинке
    imd1, contoursd1, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  #
    max1 = 0
    pok1 = 0
    for contorb1 in contoursd1:  # этот цикл ищет наибольший контур поплощади и
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


def ___(d1, w1):  # эта функция идентична предыдущей , но использует другую формулу для подсчёта среднего значения
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
                pok1 = h * (w1 - x)
                max1 = h * w
            # cv2.rectangle(d1, (x + 1, y + 1), (x + w - 1, y + h - 1), (10, 245, 0), 2)
    return pok1


def pd(sr1, sr2, kp, kd):  # пропорционально-дифференциальный регулятор
    global e, e_old, deg, color

    e = sr2 - sr1  #
    if -5 < e < 5:
        e = 0
    u = int(e * kp + (e - e_old) * kd)
    deg = u
    e_old = e  # до сюда обычный пропорциональный регулятор

    if sr1 == 0:
        deg = 55
    if sr2 == 0:
        deg = -55
    if sr1 == 0 and sr2 == 0:
        if color == "orange":
            deg = -30
        else:
            deg = 30
    if deg > 90:
        deg = 90
    if deg < -90:
        deg = -90  # различные полезные фишки


def x_road():  # функция поиска перекрёстков
    global xp1, yp1, xp2, yp2, lowb, upb, lowo, upo, color, perek, t111, flag_l, tim_per, vf, i
    datp1 = frame[yp1:yp2, xp1:xp2]
    cv2.rectangle(datp1, (xp1, yp1), (xp2, yp2), (200, 100, 100), 3)
    if color == None or color == "blue":

        hsv1 = cv2.cvtColor(datp1.copy(), cv2.COLOR_BGR2HSV)
        maskd1 = cv2.inRange(hsv1, lowb, upb)  #
        imd1, contoursb, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE,
                                                 cv2.CHAIN_APPROX_NONE)  # поиск синего перекрёстка

        for contorb1 in contoursb:
            x, y, w, h = cv2.boundingRect(contorb1)
            a1 = cv2.contourArea(contorb1)
            if a1 > 500 and t111 + 0.9 < time.time():
                if perek < 5:  # подсчёт времени для каждого участка трассы между перекрёстками
                    vrem_list[perek % 4] = round(time.time() - tim_per, 2)
                    tim_per = time.time()
                else:
                    vf = vrem_list[0] * 0.6
                color = "blue"
                LED(0, 0, 1)
                flag_l = True
                # cv2.drawContours(datp1,contoursb,0,(0,0,0),3)
                cv2.rectangle(datp1, (x, y), (x + w, y + h), (255, 0, 0), 2)  # подсчёт перекрёстков
                perek += 1
                i = 0
                t111 = time.time()

    if color == None or color == "orange":
        hsv2 = cv2.cvtColor(datp1.copy(), cv2.COLOR_BGR2HSV)  # всё тоже самое только для оранжевого
        maskd2 = cv2.inRange(hsv2, lowo, upo)  #
        imd1, contourso, hod1 = cv2.findContours(maskd2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        for contoro in contourso:
            x, y, w, h = cv2.boundingRect(contoro)
            a1 = cv2.contourArea(contoro)
            if a1 > 500 and t111 + 0.6 < time.time():
                if perek < 5:
                    vrem_list[perek % 4] = round(time.time() - tim_per, 2)
                    tim_per = time.time()
                else:
                    vf = vrem_list[0] * 0.6
                color = "orange"
                LED(1, 1, 0)
                perek += 1
                i = 0
                flag_l = True

                t111 = time.time()
                # cv2.drawContours(datp1, contourso, 0, (100, 100, 100), 3)
                cv2.rectangle(datp1, (x, y), (x + w, y + h), (0, 100, 255), 2)
    cv2.rectangle(frame, (xp1, yp1), (xp2, yp2), (0, 205, 0), 2)


def black_line():
    global x11, x12, x13, x14, x21, x22, x23, x24, y11, y12, y13, y14, y21, y22, y23, y24, w11, w12, h11, sr1, sr2
    d1 = frame[y11:y21, x11:x21]
    d2 = frame[y12:y22, x12:x22]  # забираем часть экрана длядатчиков
    d3 = frame[y13:y23, x13:x23]
    d4 = frame[y14:y24, x14:x24]

    sr1 = (__(d1) + __(d2)) // 100  # высчитываем среднее исходя из показаний датчиков
    sr2 = (___(d3, 125) + ___(d4, 250)) // 100


def LED(r, g, b):
    if r == 0:  # упрощённое управление RGB светодиодом
        GPIO.output(R, GPIO.HIGH)
    if r == 1:
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
    global ts  # относительно бесполезная функция для свечения RGB светодиодом
    if ts + delay > time.time():
        LED(1, 0, 0)
    elif ts + delay * 2 > time.time():
        LED(1, 0, 1)
    elif ts + delay * 3 > time.time():
        LED(0, 0, 1)
    elif ts + delay * 4 > time.time():
        LED(0, 1, 1)
    elif ts + delay * 5 > time.time():
        LED(0, 1, 0)
    elif ts + delay * 6 > time.time():
        LED(1, 1, 0)

    else:
        ts = time.time()


def cube_Y():  # функция поиска жёлтых кубиков
    global xk1, yk1, xk2, yk2, lowy, upy, color_Final, perek, sry, speed, hy, t_coly, timer_sp, cube, flag, \
        timerCVET, dop_per, cub
    datk1 = frame[yk1:yk2, xk1:xk2]
    x1, y1, w1, h1 = 0, 0, 0, 0
    hsv1 = cv2.cvtColor(datk1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1, lowy, upy)
    maskd2 = cv2.inRange(hsv1, lowr, upr)
    mask2 = cv2.bitwise_not(maskd2)
    mask = cv2.bitwise_and(mask2, maskd1)
    imd1, contoursk, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    max = 0

    if len(contoursk) != 0:
        timerCVET = time.time()
        for contork1 in contoursk:
            x, y, w, h = cv2.boundingRect(contork1)
            a1 = cv2.contourArea(contork1)
            if a1 > 300:
                t_coly = time.time()
                if y + h > max:
                    max = y + h
                    sry = x + w
                    hy = y + h
                    x1, y1, w1, h1 = x, y, w, h
                    timer_sp = time.time()

    else:

        if t_coly + 0.09 < time.time():
            sry = 0
            hy = 0
            x1, y1, w1, h1 = 0, 0, 0, 0

    cv2.rectangle(datk1, (x1, y1), (x1 + w1, y1 + h1), (0, 255, 255), 2)


def cube_R():  # функция поиска красных кубиков
    global xk1, yk1, xk2, yk2, lowr, upr, color_Final, perek, i, srr, speed, hr, t_colr, timer_sp, cube, flag, timerCVET, \
        flag_wr, tim_per
    datk1 = frame[yk1:yk2, xk1:xk2]

    hsv1 = cv2.cvtColor(datk1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1, lowr, upr)
    maskd2 = cv2.inRange(hsv1, lowr1, upr1)  # поиск по HSV
    mask = cv2.bitwise_or(maskd1, maskd2)
    imd1, contoursk, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    x1, y1, w1, h1 = 0, 0, 0, 0
    max = 0

    if len(contoursk) != 0:
        timerCVET = time.time()
        for contork1 in contoursk:
            x, y, w, h = cv2.boundingRect(contork1)
            a1 = cv2.contourArea(contork1)
            if a1 > 350:
                t_colr = time.time()
                LED(1, 0, 0)
                if y + h > 115 and flag_wr and perek < 5:  # запись кубиков в список
                    list[perek % 4][i] = 5
                    flag_wr = False
                    List[perek % 4][i] = round(time.time() - tim_per, 1)
                    i = 2
                if y + h < 95 and not flag_wr:
                    flag_wr = True

                if y + h > max:  # поиск наибольшего
                    max = y + h
                    srr = x + w
                    hr = y + h
                    x1, y1, w1, h1 = x, y, w, h
                    timer_sp = time.time()
            else:
                if t_colr + 0.05 < time.time():  # таймер для исчезновения кубика
                    srr = 0
                    hr = 0
                    x1, y1, w1, h1 = 0, 0, 0, 0
    else:

        if t_colr + 0.05 < time.time():
            srr = 0
            hr = 0
            x1, y1, w1, h1 = 0, 0, 0, 0

    cv2.rectangle(datk1, (x1, y1), (x1 + w1, y1 + h1), (0, 0, 255), 2)
    cv2.rectangle(frame, (xk1, yk1), (xk2, yk2), (0, 200, 200), 2)


def cube_G():  # функция поиска зеленых кубиков (за подробностями в cube_R())
    global xk1, yk1, xk2, yk2, lowg, upg, perek, srg, speed, hg, t_colg, timer_sp, color, xl1, xr1, hbl, hbr, timerCvet, flag_wg, i

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
                LED(0, 1, 0)
                if y + h > 115 and flag_wg and perek < 5:
                    list[perek % 4][i] = 3
                    flag_wg = False
                    List[perek % 4][i] = round(time.time() - tim_per, 1)
                    i = 2
                if y + h < 115 and not flag_wg:
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
        if t_colg + 0.05 < time.time():
            srg = 0
            hg = 0
            x2, y2, w2, h2 = 0, 0, 0, 0

    cv2.rectangle(datg1, (x2, y2), (x2 + w2, y2 + h2), (0, 255, 0), 2)


def process(list_c, list_v, list_C):  # нерабочая функция для обработки списка кубиков
    list_i = []
    list_n = [0, 0, 0, 0]
    list_vv = []
    list_col = []
    list_cl = []
    list_vc = []
    listvc = []
    for i in range(len(list_c)):
        a = 0
        l = list_c[i]
        for j in l:
            if j == 0:
                a += 1
        list_n[i] = a
    for i in range(len(list_n)):
        if list_n[i] == 2:
            list_i.append(i)
            list_vv.append(list_v[i])
            list_vc.append(list_C[i])
            list_cl.append(list_c[i])
    for i in range(len(list_vc)):
        l = list_vc[i]
        for j in l:
            if j != 0:
                listvc.append(j)
    for i in range(len(list_cl)):
        l = list_cl[i]
        for j in l:
            if j != 0:
                list_col.append(j)
    list_aa = []
    for i in range(len(listvc)):
        list_aa.append(round(list_vv[i] / listvc[i], 1))
    for i in range(len(list_aa)):
        if 0.6 < list_aa[i] < 1.5:
            list_aa[i] = 2
        elif 1.6 < list_aa[i] < 2.5:
            list_aa[i] = 1
        elif 2.6 < list_aa[i]:
            list_aa[i] = 0
    for i in range(len(list_c)):
        for j in list_aa:
            if list_n[i] == 2:
                list_c[i] = [0, 0, 0]
                list_c[i][j] = list_col[j]

    return list_c, list_aa


while 1:

    key = robot.get_key()

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
            LED(0, 0, 0)

        if ii == "1":
            state = 1
            ts = time.time()
            flag_start = True

    if state == 1:  # pd

        black_line()
        cube_G()
        cube_R()

        if srg == 0 and srr == 0:  # если нет кубиков

            if t_b + 0.3 > time.time():  # после кубика поворот в направлении движения
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

        elif srg != 0 and hg > hr:  # если есть зелёный куб и он ближе
            LED(0, 1, 0)
            color_Final = "Green"
            t_b = time.time()
            e = (205 + hg * 1.3) - srg  # ошибка высчитывается исходя из перспективы
            if -5 < e < 5:
                e = 0
            kp = 0.2
            kd = 0.5
            u = int(e * kp + (e - e_old) * kd)  # пропорционально-дифференциальный регулятор
            deg = 0 + u
            e_old = e

            if deg > 90:
                deg = 90
            if deg < -90:
                deg = -90
        else:  # если есть красный
            LED(1, 0, 0)
            color_Final = "Red"
            t_b = time.time()
            e = (275 - hr * 1.3) - srr

            if -5 < e < 5:
                e = 0
            kp = 0.2
            kd = 0.5
            u = int(e * kp + (e - e_old) * kd)
            deg = 0 + u
            e_old = e

            if deg > 90:
                deg = 90
            if deg < -90:
                deg = -90

        x_road()

        message = str(int(deg) + 200) + str(int(speed) + 200) + '$'  # формирование сообщения

        if perek == 12 and stope == 0:  # финиш
            stope = 1
            tf = time.time()

        if tf + vf < time.time() and stope == 1:
            state = 2
        # if perek==8 :
        #    state=3

    if state == 2:
        message = "200200$"  # остановка

    # if state==3:
    #     list,abc=process(list,vrem_list,List)
    #     state=1

    # if state == 5:  # rulevoe
    #
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

    fps1 += 1
    if time.time() > fps_time + 1:
        fps_time = time.time()
        fps = fps1
        fps1 = 0

    port.write(message.encode("utf-8"))  # отправка сообщения

    if port.in_waiting > 0 and not pyb_flag:  # ожидание и чтение сообщения
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

    cv2.rectangle(frame, (0, 0), (640, 160), (0, 0, 0), -1)  # телеметрия
    robot.text_to_frame(frame, 'fps = ' + str(fps), 3, 20, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'state = ' + str(state), 120, 20, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'message = ' + str(message), 250, 20, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'sr1-sr2= ' + str(sr1) + '-' + str(sr2) + ' color = ' + str(color), 3, 40,
                        (255, 255, 255), 1)
    robot.text_to_frame(frame, 'srr ,srg= ' + str(srr) + ' ,' + str(srg) + 'hr ,hg= ' + str(hr) + ' ,' + str(hg), 3, 60,
                        (255, 255, 255), 1)
    robot.text_to_frame(frame, 'perekryostok ' + str(perek), 3, 80, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'listt ' + str(abc), 220, 80, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'cub_list:  ' + str(list), 3, 100, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'vrlist: ' + str(List), 3, 120, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'time: ' + str(vrem_list), 3, 140, (255, 255, 255), 1)
    robot.set_frame(frame, 40)
    cv2.rectangle(frame, (xp1, yp1), (xp2, yp2), (200, 100, 100), 3)
    # if key > -1:
    #     print(str(key) + " state "+str(state))
