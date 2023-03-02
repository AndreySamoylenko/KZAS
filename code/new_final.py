import cv2
import RobotAPI as Rapi
import numpy as np
import serial
import time

port = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)
robot = Rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)

fps = 0
fps1 = 0
fps_time = 0
message = ""

# ниже идёт различное HSV для поиска цветов
# [ 91 ,135,   6] [117, 255, 164] blue
# [ 0, 87,87] [ 66 ,186 ,166] orange

lowblack = np.array([35, 67, 0])
upblack = np.array([103, 256, 29])

lowblue = np.array([94, 235, 140])
upblue = np.array([101, 256, 244])

loworange = np.array([3, 79, 113])
uporange = np.array([59, 210, 189])

lowr = np.array([0, 89, 47])
upr = np.array([6, 223, 165])

lowg = np.array([64, 200, 56])
upg = np.array([81, 255, 210])

lowr1 = np.array([164, 0, 0])
upr1 = np.array([180, 255, 255])

#     d1 = frame[220:260, 0:100]
#     d2 = frame[260:300, 0:200]
#     d3 = frame[220:260, 540:640]
#     d4 = frame[260:300, 440:640]
x1, y1, w1, h1, x2, y2, w2, h2=0, 0, 0, 0, 0, 0, 0, 0

x_line_dat = [0, 100, 0, 200, 540, 640, 440, 640]
y_line_dat = [220, 260, 260, 300, 220, 260, 260, 300]

x_perek = [280, 360]
y_perek = [310, 365]

x_cube = [80, 560]
y_cube = [80, 300]

e_old = 0  # различные переменные для ПД
speed = 0
perek = 0
color_per = "none"
kp = 0.5
kd = 5
deg = 0
u = 0
e = 0
dat1, dat2 = 0, 0
vrem_finish = 0
red_pos_x = 0
red_pos_y = 0
green_pos_x = 0
green_pos_y = 0
color_Final = 'none'
cub_pos = 0
b = 0

state = 1  # переменные состояния
stope = 0

x_road_tim = time.time()  # таймеры
tim_per = time.time()
finish_tim = time.time()
cube_r_exist = time.time()
cube_g_exist = time.time()
cube_exist_tim = time.time()

flag_start = False  # флаги
flag_l = False
flag_wg = True
flag_wr = True


vrem_list = [0, 0, 0, 0]  # список времени зон для функции x_road()
cub_col_list = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]  # список кубов по зонам
# указывается цвет ввиде цифры (переделать) ^
time_cub_list = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]  # список кубов по расположению в зоне (расчет по времени)


def wait_for_key():
    tx = '999999$'
    ii = '0'
    while ii == '0':
        port.write(tx.encode("utf-8"))
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
    xm, ym, wm, hm = 0, 0, 0, 0
    dat = cv2.GaussianBlur(d1, (5, 5), cv2.BORDER_DEFAULT)
    hsv = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)
    blur = cv2.blur(hsv, (5, 5))
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
                dat = h * (w + x)
                xm, ym, wm, hm = x, y, w, h

    return [xm + 1, ym + 1, xm + wm - 1, ym + hm - 1, dat]


def black_poisk_r(d1, w_dat):
    xm, ym, wm, hm = 0, 0, 0, 0
    dat = cv2.GaussianBlur(d1, (5, 5), cv2.BORDER_DEFAULT)
    hsv = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)
    blur = cv2.blur(hsv, (5, 5))
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
                dat = h * (w_dat - x)
                xm, ym, wm, hm = x, y, w, h

    return [xm + 1, ym + 1, xm + wm - 1, ym + hm - 1, dat]


def bortik_pro():
    global x_line_dat, y_line_dat, dat1, dat2

    d1 = frame[y_line_dat[0]:y_line_dat[1], x_line_dat[0]:x_line_dat[1]]
    d2 = frame[y_line_dat[2]:y_line_dat[3], x_line_dat[2]:x_line_dat[3]]
    d3 = frame[y_line_dat[4]:y_line_dat[5], x_line_dat[4]:x_line_dat[5]]
    d4 = frame[y_line_dat[6]:y_line_dat[7], x_line_dat[6]:x_line_dat[7]]  # забираем часть экрана длядатчиков

    dat1 = (black_poisk_l(d1)[4] + black_poisk_l(d2)[4]) // 100  # высчитываем среднее исходя из показаний датчиков
    dat2 = (black_poisk_r(d3, 125)[4] + black_poisk_r(d4, 250)[4]) // 100


def draw_rects_bortik():
    l1 = black_poisk_l(frame[y_line_dat[0]:y_line_dat[1], x_line_dat[0]:x_line_dat[1]])
    l2 = black_poisk_l(frame[y_line_dat[2]:y_line_dat[3], x_line_dat[2]:x_line_dat[3]])
    l3 = black_poisk_r(frame[y_line_dat[4]:y_line_dat[5], x_line_dat[4]:x_line_dat[5]], 100)
    l4 = black_poisk_r(frame[y_line_dat[6]:y_line_dat[7], x_line_dat[6]:x_line_dat[7]], 200)

    cv2.rectangle(frame, (x_line_dat[0], y_line_dat[0]), (x_line_dat[1], y_line_dat[1]), (0, 25, 200), 2)
    cv2.rectangle(frame, (x_line_dat[2], y_line_dat[2]), (x_line_dat[3], y_line_dat[3]), (0, 25, 200), 2)
    cv2.rectangle(frame, (x_line_dat[4], y_line_dat[4]), (x_line_dat[5], y_line_dat[5]), (0, 25, 200), 2)
    cv2.rectangle(frame, (x_line_dat[6], y_line_dat[6]), (x_line_dat[7], y_line_dat[7]), (0, 25, 200), 2)

    cv2.rectangle(frame, (x_perek[0], y_perek[0]), (x_perek[1], y_perek[1]), (0, 205, 200), 2)

    cv2.rectangle(frame[y_line_dat[0]:y_line_dat[1], x_line_dat[0]:x_line_dat[1]], (l1[0], l1[1]), (l1[2], l1[3]),
                  (10, 245, 0), 2)
    cv2.rectangle(frame[y_line_dat[2]:y_line_dat[3], x_line_dat[2]:x_line_dat[3]], (l2[0], l2[1]), (l2[2], l2[3]),
                  (10, 245, 0), 2)
    cv2.rectangle(frame[y_line_dat[4]:y_line_dat[5], x_line_dat[4]:x_line_dat[5]], (l3[0], l3[1]), (l3[2], l3[3]),
                  (10, 245, 0), 2)
    cv2.rectangle(frame[y_line_dat[6]:y_line_dat[7], x_line_dat[6]:x_line_dat[7]], (l4[0], l4[1]), (l4[2], l4[3]),
                  (10, 245, 0), 2)


def pd_regulator(d1, d2, kp, kd):  # пропорционально-дифференциальный регулятор
    global e, e_old, deg, color_per, u

    e = d2 - d1
    if -5 < e < 5:
        e = 0
    u = int(e * kp + (e - e_old) * kd)
    deg = u
    e_old = e  # до сюда обычный пропорционально-дифференциальный регулятор

    if d1 == 0:
        deg = 25
    if d2 == 0:
        deg = -25

    if d1 == 0 and d2 == 0:
        if color_per == "orange":
            deg = -30
        elif color_per == "blue":
            deg = 30

    if deg > 90:
        deg = 90
    if deg < -90:
        deg = -90  # различные полезные фишки


def x_road():  # функция поиска перекрёстков
    global color_per, perek, x_road_tim, tim_per, vrem_finish, flag_l, cub_pos
    dat = frame[y_perek[0]:y_perek[1], x_perek[0]:x_perek[1]]
    if color_per == 'none' or color_per == "blue":

        hsv = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lowblue, upblue)
        imd1, contours, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  # поиск синего перекрёстка

        for contor in contours:
            x, y, w, h = cv2.boundingRect(contor)
            a1 = cv2.contourArea(contor)
            if a1 > 500 and x_road_tim + 0.9 < time.time():
                if perek < 5:  # подсчёт времени для каждого участка трассы между перекрёстками
                    vrem_list[perek % 4] = round(time.time() - tim_per, 2)
                    tim_per = time.time()
                else:
                    vrem_finish = vrem_list[0] * 0.6
                color_per = "blue"
                flag_l = True
                cv2.rectangle(dat, (x, y), (x + w, y + h), (255, 0, 0), 2)  # подсчёт перекрёстков
                perek += 1
                cub_pos = 0
                x_road_tim = time.time()

    if color_per == "none" or color_per == "orange":
        hsv2 = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)  # всё тоже самое только для оранжевого
        maskd2 = cv2.inRange(hsv2, loworange, uporange)  #
        imd1, contourso, hod1 = cv2.findContours(maskd2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        for contoro in contourso:
            x, y, w, h = cv2.boundingRect(contoro)
            a1 = cv2.contourArea(contoro)
            if a1 > 500 and x_road_tim + 0.6 < time.time():
                if perek < 5:
                    vrem_list[perek % 4] = round(time.time() - tim_per, 2)
                    tim_per = time.time()
                else:
                    vrem_finish = vrem_list[0] * 0.6
                color_per = "orange"
                perek += 1
                cub_pos = 0
                flag_l = True
                x_road_tim = time.time()
                cv2.rectangle(dat, (x, y), (x + w, y + h), (0, 100, 255), 2)


# def cube_y():   # функция поиска жёлтых кубиков
#     global color_Final, perek, sry, speed, hy, t_coly, timer_sp, cube, flag, timerCVET, dop_per,cub
#     datk1 = frame[yk1:yk2, xk1:xk2]
#     x1, y1, w1, h1 = 0, 0, 0, 0
#     hsv1 = cv2.cvtColor(datk1, cv2.COLOR_BGR2HSV)
#     maskd1 = cv2.inRange(hsv1, lowy, upy)
#     maskd2 =cv2.inRange(hsv1  ,lowr,upr)
#     mask2=cv2.bitwise_not(maskd2)
#     mask=cv2.bitwise_and(mask2,maskd1)
#     imd1, contoursk, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
#
#     max=0
#
#     if len(contoursk)!=0:
#         timerCVET = time.time()
#         for contork1 in contoursk:
#             x, y, w, h = cv2.boundingRect(contork1)
#             a1 = cv2.contourArea(contork1)
#             if a1 > 300:
#                 t_coly= time.time()
#                 if y+h>max:
#                     max=y+h
#                     sry = x+w
#                     hy = y + h
#                     x1, y1, w1, h1 =  x, y, w, h
#                     timer_sp=time.time()
#
#     else:
#
#         if t_coly+0.09< time.time():
#             sry = 0
#             hy = 0
#             x1, y1, w1, h1 = 0, 0, 0, 0
#
#
#
#     cv2.rectangle(datk1, (x1, y1), (x1 + w1, y1 + h1), (0, 255,255), 2)


def cube_r():  # функция поиска красных кубиков
    global color_Final, perek, red_pos_x, cub_pos, red_pos_y, cube_r_exist, timer_sp, cube, flag, timerCVET, flag_wr,\
        x1, y1, w1, h1, tim_per
    datk1 = frame[y_cube[0]:y_cube[1], x_cube[0]:x_cube[1]]

    hsv1 = cv2.cvtColor(datk1, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv1, lowr, upr)
    mask2 = cv2.inRange(hsv1, lowr1, upr1)  # поиск по HSV
    mask = cv2.bitwise_or(mask1, mask2)
    imd1, contours, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    x1, y1, w1, h1 = 0, 0, 0, 0
    max = 0

    if len(contours) != 0:
        timerCVET = time.time()
        for contor in contours:
            x, y, w, h = cv2.boundingRect(contor)
            area = cv2.contourArea(contor)
            if area > 350:
                cube_r_exist = time.time()
                if y + h > 115 and flag_wr and perek < 5:  # запись кубиков в список
                    cub_col_list[perek % 4][cub_pos] = 5
                    flag_wr = False
                    time_cub_list[perek % 4][cub_pos] = round(time.time() - tim_per, 1)
                    cub_pos = 2
                if y + h < 95 and not flag_wr:
                    flag_wr = True

                if y + h > max:  # поиск наибольшего
                    max = y + h
                    red_pos_x = x + w
                    red_pos_x = y + h
                    x1, y1, w1, h1 = x, y, w, h
                    timer_sp = time.time()
            else:
                if cube_r_exist + 0.05 < time.time():  # таймер для исчезновения кубика
                    red_pos_x = 0
                    red_pos_x = 0
                    x1, y1, w1, h1 = 0, 0, 0, 0

    else:
        if cube_r_exist + 0.05 < time.time():
            red_pos_x = 0
            red_pos_x = 0
            x1, y1, w1, h1 = 0, 0, 0, 0

    cv2.rectangle(datk1, (x1, y1), (x1 + w1, y1 + h1), (0, 0, 255), 2)



def cube_g():  # функция поиска зеленых кубиков (за подробностями в cube_R())
    global green_pos_x, speed, green_pos_y, cube_g_exist, timer_sp, color_per, timerCvet, flag_wg, cub_pos,\
        x2, y2, w2, h2

    datg1 = frame[y_cube[0]:y_cube[1], x_cube[0]:x_cube[1]]
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
                cube_g_exist = time.time()
                if y + h > 115 and flag_wg and perek < 5:
                    cub_col_list[perek % 4][cub_pos] = 3
                    flag_wg = False
                    time_cub_list[perek % 4][cub_pos] = round(time.time() - tim_per, 1)
                    cub_pos = 2
                if y + h < 115 and not flag_wg:
                    flag_wg = True

                if y + h > max:
                    max = y + h
                    green_pos_x = x
                    green_pos_y = y + h
                    x2, y2, w2, h2 = x, y, w, h
                    timer_sp = time.time()
            else:
                if cube_g_exist + 0.07 < time.time():
                    green_pos_x = 0
                    green_pos_y = 0
                    x2, y2, w2, h2 = 0, 0, 0, 0
    else:
        if cube_g_exist + 0.05 < time.time():
            green_pos_x = 0
            green_pos_y = 0
            x2, y2, w2, h2 = 0, 0, 0, 0




def state_changer():
    global state
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

wait_for_key()

while 1:
    # state_changer()
    frame = robot.get_frame(wait_new_frame=1)

    if state == 1:  # езда
        cube_g()
        cube_r()

        if green_pos_x == 0 and red_pos_x == 0:  # если нет кубиков
            bortik_pro()  # ищем бортики
            if cube_exist_tim + 0.3 > time.time():  # после кубика поворот в направлении движения
                if color_per == "orange" and color_Final == "Green":
                    dat2 = 0
                elif color_Final == "Red" and color_per == "blue":
                    dat1 = 0
                b = 1
            elif b == 1:
                color_Final = "none"
                b = 0
            pd_regulator(dat1, dat2, kp, kd)

        elif green_pos_x != 0 and green_pos_y > red_pos_x:  # если есть зелёный куб и он ближе
            color_Final = "Green"
            cube_exist_tim = time.time()
            # e = (205 + green_pos_y * 1.3) - green_pos_x  # ошибка высчитывается исходя из перспективы
            pd_regulator((205 + green_pos_y * 1.3), green_pos_x, 0.2, 0.5)
            # if -5 < e < 5:
            #     e = 0
            # kp = 0.2
            # kd = 0.5
            # u = int(e * kp + (e - e_old) * kd)  # пропорционально-дифференциальный регулятор
            # deg = 0 + u
            # e_old = e

            if deg > 90:
                deg = 90
            if deg < -90:
                deg = -90
        else:  # если есть красный
            color_Final = "Red"
            cube_exist_tim = time.time()
            e = (275 - red_pos_x * 1.3) - red_pos_x

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

    if state == 2:  # стоп
        deg = 0
        speed = 0

    if perek == 12 and stope == 0:  # финиш
        stope = 1
        finish_tim = time.time()

    if finish_tim + vrem_finish < time.time() and stope == 1:
        state = 2

    fps1 += 1
    if time.time() > fps_time + 1:
        fps_time = time.time()
        fps = fps1
        fps1 = 0

    port.write(message.encode("utf-8"))  # отправка сообщения

    draw_rects_bortik()
    cv2.rectangle(frame, (x_cube[0], y_cube[0]), (x_cube[1], y_cube[1]), (0, 200, 200), 2)
    cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (5, 5, 255), 2)
    cv2.rectangle(frame, (x2, y2), (x2 + w2, y2 + h2), (5, 255, 5), 2)

    robot.text_to_frame(frame, 'fps = ' + str(fps), 3, 20, (255, 255, 255), 1)  # телеметрия
    robot.text_to_frame(frame, 'state = ' + str(state), 120, 20, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'message = ' + str(message), 250, 20, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'sr1-sr2= ' + str(dat1) + '-' + str(dat2) + ' color_per = ' + str(color_per), 3, 40,
                        (255, 255, 255), 1)
    robot.text_to_frame(frame, 'red_pos_x ,green_pos_x= ' + str(red_pos_x) + ' ,' + str(
        green_pos_x) + 'red_pos_x ,green_pos_y= ' + str(
        red_pos_x) + ' ,' + str(green_pos_y), 3, 60, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'perekryostok ' + str(perek), 3, 80, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'cub_list:  ' + str(cub_col_list), 3, 100, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'vrlist: ' + str(time_cub_list), 3, 120, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'time: ' + str(vrem_list), 3, 140, (255, 255, 255), 1)
    robot.set_frame(frame, 40)
    # if key > -1:
    #     print(str(key) + " state "+str(state))
