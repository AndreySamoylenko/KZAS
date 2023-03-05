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
red = 0
gren = 0
blu = 0
# [5, 63, 78] [39, 173, 186] - ораньжевый hsv новый
# ниже идёт различное HSV для поиска цветов
# [81, 55, 50] [113, 256, 253] blue
# [ 0, 87,87] [ 66 ,186 ,166] orange

lowblack = np.array([35, 67, 0])
upblack = np.array([103, 256, 29])

lowblue = np.array([99, 68, 16])
upblue = np.array([109, 197, 151])

loworange = np.array([5, 63, 78])
uporange = np.array([39, 173, 186])

lowr = np.array([0, 89, 47])
upr = np.array([6, 223, 165])

lowg = np.array([64, 200, 56])
upg = np.array([81, 255, 210])

#     d1 = frame[220:260, 0:100]
#     d2 = frame[260:300, 0:200]
#     d3 = frame[220:260, 540:640]
#     d4 = frame[260:300, 440:640]

x_line_dat = [0, 120, 0, 200, 520, 640, 440, 640]  #
y_line_dat = [230, 270, 270, 310, 230, 270, 270, 310]

x_perek = [280, 360]
y_perek = [280, 325]

e_old = 0  # различные переменные для ПД
speed = 60
perek = 0
color_per = "none"
kp = 0.25
kd = 5
deg = 0
u = 0
e = 0
dat1, dat2 = 0, 0
vrem_finish = 0

state = 1  # переменные состояния
stope = 0

x_road_tim = time.time()  # таймеры
tim_per = time.time()
finish_tim = time.time()

flag_start = False  # флаги
flag_l = False

vrem_list = [0, 0, 0, 0]  # список времени зон для функции x_road()


def wait_for_key():
    tx = '999999999999999999999$'
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
    dat2 = (black_poisk_r(d3, x_line_dat[1])[4] + black_poisk_r(d4, x_line_dat[3])[4]) // 100


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


def pd_regulator(d1, d2):  # пропорционально-дифференциальный регулятор
    global e, e_old, deg, color_per, u

    e = d2 - d1
    if -10 < e < 10:
        e = 0
    u = int(e * kp + (e - e_old) * kd)
    deg = u
    e_old = e  # до сюда обычный пропорционально-дифференциальный регулятор

    if d1 == 0:
        deg = 30
    if d2 == 0:
        deg = -30

    if d1 == 0 and d2 == 0:
        if color_per == "orange":
            deg = -32
        elif color_per == "blue":
            deg = 32

    if deg > 90:
        deg = 90
    if deg < -90:
        deg = -90  # различные полезные фишки


def x_road():  # функция поиска перекрёстков
    global x_perek, y_perek, lowblue, upblue, loworange, uporange, color_per, perek, x_road_tim, tim_per, vrem_finish, \
        flag_l
    dat = frame[y_perek[0]:y_perek[1], x_perek[0]:x_perek[1]]
    if color_per == 'none' or color_per == "blue":

        hsv = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lowblue, upblue)  #
        imd1, contours, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  # поиск синего перекрёстка

        for contor in contours:
            x, y, w, h = cv2.boundingRect(contor)
            a1 = cv2.contourArea(contor)
            if a1 > 500 and x_road_tim + 0.9 < time.time():
                if perek < 5:  # подсчёт времени для каждого участка трассы между перекрёстками
                    vrem_list[perek % 4] = round(time.time() - tim_per, 2)
                    tim_per = time.time()
                else:
                    vrem_finish = vrem_list[0] * 0.7
                color_per = "blue"
                flag_l = True
                cv2.rectangle(dat, (x, y), (x + w, y + h), (255, 0, 0), 2)  # подсчёт перекрёстков
                perek += 1
                x_road_tim = time.time()

    if color_per == 'none' or color_per == "orange":
        hsv2 = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)  # всё тоже самое только для оранжевого
        maskd2 = cv2.inRange(hsv2, loworange, uporange)  #
        imd1, contourso, hod1 = cv2.findContours(maskd2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        for contoro in contourso:
            x, y, w, h = cv2.boundingRect(contoro)
            a1 = cv2.contourArea(contoro)
            if a1 > 500 and x_road_tim + 0.7 < time.time():
                if perek < 5:
                    vrem_list[perek % 4] = round(time.time() - tim_per, 2)
                    tim_per = time.time()
                else:
                    vrem_finish = vrem_list[0] * 0.6
                color_per = "orange"
                perek += 1
                flag_l = True

                x_road_tim = time.time()
                cv2.rectangle(dat, (x, y), (x + w, y + h), (0, 100, 255), 2)


def print_message(sp, dg, r=0, g=0, b=0):
    lst = [str(sp + 200), str(dg + 200), str(r + 200), str(g + 200), str(b + 200), '$']
    strin = ",".join(lst)
    port.write(strin.encode("utf-8"))


wait_for_key()

while 1:
    frame = robot.get_frame(wait_new_frame=1)
    if state == 1:
        red, gren, blu = 0, 0, 0
        bortik_pro()
        pd_regulator(dat1, dat2)
        x_road()
        if x_road_tim + 0.5 > time.time():
            if color_per == 'orange':
                red = 70
                gren = 45
            if color_per == 'blue':
                blu = 80
                gren = 20

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

    draw_rects_bortik()

    print_message(speed, deg, red, gren, blu)
    port.write(message.encode("utf-8"))

    robot.text_to_frame(frame, 'fps = ' + str(fps), 50, 20)
    robot.text_to_frame(frame, dat1, 0, 140)
    robot.text_to_frame(frame, dat2, 600, 140)
    robot.text_to_frame(frame, deg, 300, 200)
    robot.text_to_frame(frame, color_per + " " + str(perek), 265, 400)
    robot.set_frame(frame, 40)
