import cv2
import RobotAPI as Rapi
import numpy as np
import serial
import time

# define all variable
port = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)
robot = Rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)

fps = 0
fps1 = 0
fps_time = 0
message = ""
red = 0
green = 0
blue = 0
# [5, 63, 78] [39, 173, 186] - ораньжевый hsv новый
# ниже идёт различное HSV для поиска цветов
# [81, 55, 50] [113, 256, 253] blue
# [ 0, 87,87] [ 66 ,186 ,166] orange

lowblack = np.array([35, 67, 0])
upblack = np.array([103, 256, 29])

lowblue = np.array([101, 66, 0])
upblue = np.array([123, 255, 255])

loworange = np.array([5, 63, 78])
uporange = np.array([39, 173, 186])

lowred = np.array([0, 89, 47])
upred = np.array([6, 223, 165])

lowgreen = np.array([64, 200, 56])
upgeen = np.array([81, 255, 210])

#     d1 = frame[220:260, 0:100]
#     d2 = frame[260:300, 0:200]
#     d3 = frame[220:260, 540:640]
#     d4 = frame[260:300, 440:640]
# размеры всей области 640 на 480

x_line_dat = [0, 220, 420, 640]
y_line_dat = [210, 270, 210, 270]

x_cross = [280, 360]
y_cross = [280, 325]

x_zonecheck_dat = [310, 330]
y_zonecheck_dat = [50, 300]

e_old = 0  # различные переменные для ПД
speed = 100
cross = 0
color_line = "none"
kp = 0.2
kd = 1
degree = 0
u = 0
e = 0
dat1, dat2 = 0, 0
time_finish = 0

state = 2  # переменные состояния
stop = 0

search_cross_time = time.time()  # таймеры
cross_time = time.time()
finish_tim = time.time()

flag_start = False  # флаги
flag_l = False

time_list = [0, 0, 0, 0]  # список времени зон для функции search_cross()


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


def black_search_left(d1, area_min=500):
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
        if area > area_min:
            if max1 < h * w:
                max1 = h * w
                dat = (w + x)
                xm, ym, wm, hm = x, y, w, h

    return [xm + 1, ym + 1, xm + wm - 1, ym + hm - 1, dat]


def black_search_right(d1, w_dat):
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
                dat = (w_dat - x)
                xm, ym, wm, hm = x, y, w, h

    return [xm + 1, ym + 1, xm + wm - 1, ym + hm - 1, dat]


def detect_line_pro():
    global x_line_dat, y_line_dat, dat1, dat2

    d1 = frame[y_line_dat[0]:y_line_dat[1], x_line_dat[0]:x_line_dat[1]]
    d2 = frame[y_line_dat[2]:y_line_dat[3], x_line_dat[2]:x_line_dat[3]]
    # забираем часть экрана длядатчиков

    # высчитываем среднее исходя из показаний датчиков
    dat1 = (black_search_left(d1)[4])
    dat2 = (black_search_right(d2, x_line_dat[1])[4])


def draw_contour_line():
    l1 = black_search_left(frame[y_line_dat[0]:y_line_dat[1], x_line_dat[0]:x_line_dat[1]])
    l2 = black_search_right(frame[y_line_dat[2]:y_line_dat[3], x_line_dat[2]:x_line_dat[3]], 200)

    cv2.rectangle(frame, (x_line_dat[0], y_line_dat[0]), (x_line_dat[1], y_line_dat[1]), (0, 25, 200), 2)
    cv2.rectangle(frame, (x_line_dat[2], y_line_dat[2]), (x_line_dat[3], y_line_dat[3]), (0, 25, 200), 2)

    cv2.rectangle(frame, (x_cross[0], y_cross[0]), (x_cross[1], y_cross[1]), (0, 205, 200), 2)

    cv2.rectangle(frame[y_line_dat[0]:y_line_dat[1], x_line_dat[0]:x_line_dat[1]], (l1[0], l1[1]), (l1[2], l1[3]),
                  (10, 245, 0), 2)
    cv2.rectangle(frame[y_line_dat[2]:y_line_dat[3], x_line_dat[2]:x_line_dat[3]], (l2[0], l2[1]), (l2[2], l2[3]),
                  (10, 245, 0), 2)


def zone_check():
    # отрисовка датчика для определения узкой зоны поля(потом убрать)
    cv2.rectangle(frame, (x_zonecheck_dat[0], y_zonecheck_dat[0]), (x_zonecheck_dat[1], y_zonecheck_dat[1]),
                  (200, 0, 0), 1)
    l = black_search_left(frame[y_zonecheck_dat[0]:y_zonecheck_dat[1], x_zonecheck_dat[0]:x_zonecheck_dat[1]], 400)
    cv2.rectangle(frame[y_zonecheck_dat[0]:y_zonecheck_dat[1], x_zonecheck_dat[0]:x_zonecheck_dat[1]], (l[0], l[1]),
                  (l[2], l[3]), (10, 245, 0), 2)
    robot.text_to_frame(frame, l[3], 200, 200)


def pd_regulator(d1, d2):  # пропорционально-дифференциальный регулятор
    global e, e_old, degree, color_line, u

    e = d2 - d1
    if -5 < e < 5:
        e = 0
    u = int(e * kp + (e - e_old) * kd)
    degree = u
    e_old = e  # до сюда обычный пропорционально-дифференциальный регулятор

    if d1 == 0:
        degree = 25
    if d2 == 0:
        degree = -25

    if d1 == 0 and d2 == 0:
        if color_line == "orange":
            degree = -30
        elif color_line == "blue":
            degree = 30

    # if degree > 90:
    #     degree = 90
    # if degree < -90:
    #     degree = -90


def search_cross():  # функция поиска перекрёстков
    global x_cross, y_cross, lowblue, upblue, loworange, uporange, color_line, cross, search_cross_time, cross_time, \
        flag_l, time_finish
    dat = frame[y_cross[0]:y_cross[1], x_cross[0]:x_cross[1]]
    if color_line == 'none' or color_line == "blue":

        hsv = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lowblue, upblue)  #
        imd1, contours, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  # поиск синего перекрёстка

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            a1 = cv2.contourArea(contour)
            if a1 > 500 and search_cross_time + 0.9 < time.time():
                if cross < 5:  # подсчёт времени для каждого участка трассы между перекрёстками
                    time_list[cross % 4] = round(time.time() - cross_time, 2)
                    cross_time = time.time()
                else:
                    time_finish = time_list[0] * 0.7
                color_line = "blue"
                flag_l = True
                cv2.rectangle(dat, (x, y), (x + w, y + h), (255, 0, 0), 2)  # подсчёт перекрёстков
                cross += 1
                search_cross_time = time.time()

    if color_line == 'none' or color_line == "orange":
        hsv2 = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)  # всё тоже самое только для оранжевого
        mask_d2 = cv2.inRange(hsv2, loworange, uporange)  #
        imd1, contours1, hod1 = cv2.findContours(mask_d2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        for contour1 in contours1:
            x, y, w, h = cv2.boundingRect(contour1)
            a1 = cv2.contourArea(contour1)
            if a1 > 500 and search_cross_time + 0.7 < time.time():
                if cross < 5:
                    time_list[cross % 4] = round(time.time() - cross_time, 2)
                    cross_time = time.time()
                else:
                    time_finish = time_list[0] * 0.6
                color_line = "orange"
                cross += 1
                flag_l = True

                search_cross_time = time.time()
                cv2.rectangle(dat, (x, y), (x + w, y + h), (0, 100, 255), 2)


def print_message(sp, dg, r=0, g=0, b=0):
    lst = [str(sp + 200), str(dg + 200), str(r + 200), str(g + 200), str(b + 200), '$']
    string = ",".join(lst)
    port.write(string.encode("utf-8"))


wait_for_key()

while 1:
    frame = robot.get_frame(wait_new_frame=1)
    key = robot.get_key()
    if key != -1:
        if key == 50:
            state = 2
        if key == 49:
            state = 1
        if key == 51:
            state = 3

    if state == 1:
        speed = 100
        red, green, blue = 0, 0, 0
        detect_line_pro()
        pd_regulator(dat1, dat2)
        search_cross()
        if search_cross_time + 0.5 > time.time():
            if color_line == 'orange':
                red = 70
                green = 45
            if color_line == 'blue':
                blue = 80
                green = 20

    if state == 2:  # stop
        degree = 0
        speed = 0

    if state == 3:
        if key == -1:
            pass
        elif key == 87:  # это W
            speed = 30

        elif key == 83:  # это S
            speed = -30

        elif key == 65:  # это A
            degree += 3

        elif key == 68:  # это D
            degree -= 3

        elif key == 32:
            speed = 0
            degree = 0

        if degree > 40:
            degree = 40
        if degree < - 40:
            degree = -40

    if cross == 12 and stop == 0:  # finish
        stop = 1
        finish_tim = time.time()

    if finish_tim + time_finish < time.time() and stop == 1:
        state = 2

    fps1 += 1
    if time.time() > fps_time + 1:
        fps_time = time.time()
        fps = fps1
        fps1 = 0

    draw_contour_line()

    # zone_check()

    print_message(speed, degree, red, green, blue)
    port.write(message.encode("utf-8"))

    robot.text_to_frame(frame, 'fps = ' + str(fps), 50, 20)
    robot.text_to_frame(frame, dat1, 0, 140)
    robot.text_to_frame(frame, dat2, 600, 140)
    robot.text_to_frame(frame, degree, 300, 200)
    robot.text_to_frame(frame, speed, 300, 220)
    robot.text_to_frame(frame, color_line + " " + str(cross), 265, 400)
    robot.set_frame(frame, 40)
