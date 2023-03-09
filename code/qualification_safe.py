import cv2  # подключаем библиотеку компьютерного зрения
import RobotAPI as Rapi  # подключаем вспомогательную библиотеку для работы с распберри
import numpy as np  # подключаем библиотеку numpy
import serial  # подключаем библиотеку Serial для общения между распберри и пайбордом
import time  # подключаем библиотеку time для работы с таймерами

port = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)
robot = Rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)

fps = 0
fps1 = 0
fps_time = 0

red = 0
green = 0
blue = 0

#  HSV для поиска цветов

lowblack = np.array([35, 67, 0])
upblack = np.array([103, 256, 29])

lowblue = np.array([84, 80, 34])
upblue = np.array([146, 256, 242])

loworange = np.array([5, 63, 78])
uporange = np.array([39, 173, 186])

lowred = np.array([0, 89, 47])
upred = np.array([6, 223, 165])

lowgreen = np.array([64, 200, 56])
upgeen = np.array([81, 255, 210])

x_line_dat = [0, 220, 420, 640]  # координаты для датчиков линии
y_line_dat = [210, 270, 210, 270]

x_cross = [280, 360]  # координаты для датчика перекрёстка (оранжевой или синей линии)
y_cross = [280, 325]

e_old = 0  # различные переменные для ПД
kp = 0.2  # коэффициент пропорциональной составляющей
kd = 1  # коэффициент дифференциальной составляющей
u = 0  # управляющее воздействие
e = 0  # ошибка (отклонение)
dat1, dat2 = 0, 0  # показания датчиков линии

cross = 0  # счётчик перекрёсков
color_line = "none"  # цвет перекрёстка (оранжевый или синий) используется для определения направления движения

time_finish = 0  # время для финишной зоны засечённое с помощью функции search_cross()

state = 1  # переменные состояния
stop = False

search_cross_time = time.time()  # таймеры
cross_time = time.time()
finish_tim = time.time()

flag_start = False  # флаги
flag_l = False

time_list = [0, 0, 0, 0]  # список времени зон получаемых из функции search_cross()

message = ""  # сообщение формируемое функцией print_message()

speed = 70   # скорость
degree = 0  # угол поворота сервомоторчика


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
            if a1 > 500 and search_cross_time + 0.9 < time.time():
                if cross < 5:
                    time_list[cross % 4] = round(time.time() - cross_time, 2)
                    cross_time = time.time()
                else:
                    time_finish = time_list[0] * 0.7
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

    if state == 1:
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

    if cross == 12 and not stop:  # если проехали 12 перекрёстков и флаг опущен
        stop = True  # поднимаем флаг
        finish_tim = time.time()  # засекаем время

    if finish_tim + time_finish < time.time() and stop == 1:
        # если с момента поднятия флага прошло время необходимое для проезда в центр зоны
        state = 2  # переходим в состояние "стоп"

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
    robot.text_to_frame(frame, 'degree = ' + str(degree), 250, 200)
    robot.text_to_frame(frame, 'speed = ' + str(speed), 260, 220)
    robot.text_to_frame(frame, color_line + " " + str(cross), 265, 400)

    robot.set_frame(frame, 40)