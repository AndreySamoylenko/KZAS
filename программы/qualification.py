import cv2                  # подключаем библиотеку компьютерного зрения
import RobotAPI as Rapi     # подключаем вспомогательную библиотеку для работы с распберри
import numpy as np          # подключаем библиотеку numpy
import serial               # подключаем библиотеку Serial для общения между распберри и пайбордом
import time                 # подключаем библиотеку time для работы с таймерами

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

lowblack = np.array([0, 75,  0])   # черный
upblack = np.array([180, 255,  50])

lowblue = np.array([84, 80, 34])    # синий
upblue = np.array([146, 255, 242])

loworange = np.array([5, 63, 78])   # оранжевый
uporange = np.array([39, 173, 186])

lowred = np.array([0, 89, 47])      # красный
upred = np.array([6, 223, 165])

lowgreen = np.array([64, 200, 56])  # зелёный
upgreen = np.array([81, 255, 210])

# координаты областей интереса

x_line_dat = [0, 220, 420, 640]  # координаты для датчиков линии
y_line_dat = [240, 270, 240, 270]

x_cross = [280, 360]  # координаты для датчика перекрёстка (оранжевой или синей линии)
y_cross = [320, 360]


# различные переменные для ПД

e_old = 0   # значение предыдущей ошибки для подсчёта дифференциальной составляющей
kp = 0.25    # коэффициент пропорциональной составляющей
kd = 0.2    # коэффициент дифференциальной составляющей
u = 0       # управляющее воздействие
e = 0       # ошибка (отклонение)
dat1, dat2 = 0, 0  # показания датчиков линии

cross = 0               # счётчик перекрёсков
color_line = "none"     # цвет перекрёстка (оранжевый или синий) используется для определения направления движения

time_finish = 0         # время для финишной зоны засечённое с помощью функции search_cross()

state = 1  # переменная состояния

stop_flag = False  # флаг финиша

# таймеры
search_cross_time = time.time()     # таймер для предотвращения множественных срабатываний датчика перекрёстка
cross_time = time.time()            # таймер для засекания времени между перекрёстками
finish_tim = time.time()            # таймер для финиша в середине зоны
stop_timer = time.time()            # таймер активного торможения

time_list = [0, 0, 0, 0]  # список времени зон получаемых из функции search_cross()

speed = 55  # скорость
degree = 0  # угол поворота сервопривода


def wait_for_key():
    tx = '999999999999999999999$'           # сообщение об ожидании кнопки
    ii = '0'                                # сообщение с микроконтроллера
    while ii == '0':                        # пока сообщение равно "0" (кнопка не нажата)
        port.write(tx.encode("utf-8"))      # отправить сообщение об ожидании кнопки
        if port.in_waiting > 0:             # если что-то пришло
            ii = ""                         # очищаем сообщение
            t = time.time()                 # засекаем время
            while 1:                        # всегда
                a = str(port.read(), "utf-8")  # получаем символ из сообщения
                if a != '$':                # если символ не стоп-символ
                    ii += a                 # прибавить символ к сообщению
                else:                       # иначе(пришёл стоп-символ)
                    break                   # прекращаем читать сообщение
                if t + 0.02 < time.time():  # если вышел таймаут
                    break                   # прекращаем читать сообщение


def black_search_left(d1):
    xm, ym, wm, hm = 0, 0, 0, 0
    dat = cv2.GaussianBlur(d1, (5, 5), cv2.BORDER_DEFAULT)  # размытие фрагмента изображения
    hsv = cv2.cvtColor(dat, cv2.COLOR_BGR2HSV)       # перевод в цветовую модель HSV
    mask = cv2.inRange(hsv, lowblack, upblack)              # создание маски
    blur = cv2.blur(mask, (5, 5))                           # размытие маски

    imd1, contours, hod1 = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  # функция поиска контуров

    dat = 0
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)  # координаты текущего контура
        area = cv2.contourArea(contour)         # площадь текущего контура
        if area > 250:                          # фильтр по минимальной площади контура
            if dat < w + x:                     # поиск максимального по площади контура
                dat = (w + x)                   # перезапись показаний датчиков
                xm, ym, wm, hm = x, y, w, h     # запись координат наибольшего контура

    return [xm + 1, ym + 1, xm + wm - 1, ym + hm - 1, int(dat/220*100)]


def black_search_right(d1, w_dat):
    xm, ym, wm, hm = 0, 0, 0, 0
    dat = cv2.GaussianBlur(d1, (5, 5), cv2.BORDER_DEFAULT)  # размытие фрагмента изображения
    hsv = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)       # перевод в цветовую модель HSV
    mask = cv2.inRange(hsv, lowblack, upblack)              # создание маски
    blur = cv2.blur(mask, (5, 5))                           # размытие маски

    imd1, contours, hod1 = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  # функция поиска контуров
    dat = 0
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)  # координаты текущего контура
        area = cv2.contourArea(contour)         # площадь текущего контура
        if area > 250:                          # фильтр по минимальной площади контура
            if dat < w_dat - x:                 # поиск максимального по площади контура
                dat = (w_dat - x)               # перезапись показаний датчиков
                xm, ym, wm, hm = x, y, w, h     # запись координат наибольшего контура

    return [xm + 1, ym + 1, xm + wm - 1, ym + hm - 1, int(dat/220*100)]


def detect_line_pro():
    global x_line_dat, y_line_dat, dat1, dat2

    # забираем часть экрана для датчиков
    d1 = frame[y_line_dat[0]:y_line_dat[1], x_line_dat[0]:x_line_dat[1]]
    d2 = frame[y_line_dat[2]:y_line_dat[3], x_line_dat[2]:x_line_dat[3]]

    # высчитываем показания датчиков
    dat1 = (black_search_left(d1)[4])
    dat2 = (black_search_right(d2, x_line_dat[1])[4])


def draw_contour_line():  # функция для отображения датчиков и контуров
    # получаем координаты контуров
    l1 = black_search_left(frame[y_line_dat[0]:y_line_dat[1], x_line_dat[0]:x_line_dat[1]])
    l2 = black_search_right(frame[y_line_dat[2]:y_line_dat[3], x_line_dat[2]:x_line_dat[3]], 200)
    # отрисовываем границы датчиков
    cv2.rectangle(frame, (x_line_dat[0], y_line_dat[0]), (x_line_dat[1], y_line_dat[1]), (0, 25, 200), 2)
    cv2.rectangle(frame, (x_line_dat[2], y_line_dat[2]), (x_line_dat[3], y_line_dat[3]), (0, 25, 200), 2)
    cv2.rectangle(frame, (x_cross[0], y_cross[0]), (x_cross[1], y_cross[1]), (0, 205, 200), 2)
    # отрисовываем прямоугольники вокруг контуров
    cv2.rectangle(frame[y_line_dat[0]:y_line_dat[1], x_line_dat[0]:x_line_dat[1]], (l1[0], l1[1]), (l1[2], l1[3]),
                  (10, 245, 0), 2)
    cv2.rectangle(frame[y_line_dat[2]:y_line_dat[3], x_line_dat[2]:x_line_dat[3]], (l2[0], l2[1]), (l2[2], l2[3]),
                  (10, 245, 0), 2)


def pd_regulator(d1, d2):  # пропорционально-дифференциальный регулятор
    global e, e_old, degree, color_line, u

    e = d2 - d1     # вычисяем отклонение
    if -5 < e < 5:  # если отклонение небольшое
        e = 0       # приравниваем к нулю
    u = int(e * kp + (e - e_old) * kd)      # вычисляем управляющее воздействие по формуле ПД регулятора
    degree = u      # приравниваем угол к управляющему воздействию
    e_old = e       # запоминаем предыдущую ошибку

    if d1 == 0:     # если нет бортика поворачиваем в сторону где он должен быть
        degree = 30
    if d2 == 0:
        degree = -30

    if d1 == 0 and d2 == 0:     # если нет обоих бортиков поворачиваем в направлении движения
        if color_line == "orange":
            degree = -35
        elif color_line == "blue":
            degree = 35

    if d1 > 180 and d2 > 180:   # если оба датчика показывают больше 180
        if color_line == "orange":  # значит мы едем в бортик и надо срочно выруливать в направлении движения
            degree = -60
        elif color_line == "blue":
            degree = 60


def search_cross():  # функция поиска перекрёстков
    global x_cross, y_cross, lowblue, upblue, loworange, uporange, color_line, cross, search_cross_time, cross_time, \
         time_finish
    dat = frame[y_cross[0]:y_cross[1], x_cross[0]:x_cross[1]]
    if color_line == 'none' or color_line == "blue":

        hsv = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lowblue, upblue)  #
        imd1, contours, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  # поиск синего перекрёстка

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            a1 = cv2.contourArea(contour)
            if a1 > 400 and search_cross_time + 0.9 < time.time():
                if cross < 5:  # подсчёт времени для каждого участка трассы между перекрёстками
                    time_list[cross % 4] = round(time.time() - cross_time, 2)
                    cross_time = time.time()
                else:
                    time_finish = time_list[0] * 0.6
                color_line = "blue"
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
            if a1 > 400 and search_cross_time + 0.9 < time.time():
                if cross < 5:
                    time_list[cross % 4] = round(time.time() - cross_time, 2)
                    cross_time = time.time()
                else:
                    time_finish = time_list[0] * 0.6
                color_line = "orange"
                cross += 1

                search_cross_time = time.time()
                cv2.rectangle(dat, (x, y), (x + w, y + h), (0, 100, 255), 2)


def print_message(sp, dg, r=0, g=0, b=0):
    lst = [str(sp + 200), str(dg + 200), str(r + 200), str(g + 200), str(b + 200), '$']  # формируем сообщение
    string = ",".join(lst)      # переводим сообщение в строку
    port.write(string.encode("utf-8"))  # отправляем сообщение


wait_for_key()  # ожидание кнопки

while 1:
    frame = robot.get_frame(wait_new_frame=1)

    if state == 1:  # движение
        red, green, blue = 0, 0, 0  # переменные хранящие состояние RGB светодиода
        detect_line_pro()           # функция удобного считывания датчиков
        pd_regulator(dat1, dat2)    # ПД регулятор
        search_cross()              # считывание перекрёстков
        if search_cross_time + 0.5 > time.time():
            # если с момента засечения перекрёстка прошло менее 0.5 секунд
            if color_line == 'orange':      # если цвет перекрёстка оранжевый зажечь оранжевый
                red = 84
                green = 54
            if color_line == 'blue':        # если цвет перекрёстка синий зажечь синий
                blue = 80
                green = 20

    if state == 2:  # активное торможение
        if stop_timer + 0.1 > time.time():  # если прошло менее 0.1 секунд
            speed = -100                    # гасим инерцию
        else:                               # иначе
            state = 3                       # переходим в состояние стоп

    if state == 3:  # стоп
        degree = 0
        speed = 0

    if cross == 12 and not stop_flag:   # если проехали 12 перекрёстков и флаг опущен
        stop_flag = True                # поднимаем флаг
        finish_tim = time.time()        # засекаем время
        cross = 13

    if finish_tim + time_finish < time.time() and stop_flag:
        # если с момента поднятия флага прошло время
        # необходимое для проезда в центр зоны
        state = 2  # переходим в состояние активного торможения
        stop_timer = time.time()
        stop_flag = False

    fps1 += 1               # подсчёт кадров в секунду
    if time.time() > fps_time + 1:
        fps_time = time.time()
        fps = fps1
        fps1 = 0

    draw_contour_line()     # функция отрисовки контуров

    print_message(speed, degree, red, green, blue)  # формирование и отправка сообщения

    robot.text_to_frame(frame, 'fps = ' + str(fps), 50, 20)   # телеметрия
    robot.text_to_frame(frame, dat1, 0, 260)
    robot.text_to_frame(frame, dat2, 600, 260)
    robot.text_to_frame(frame, 'degree = ' + str(degree), 250, 200)
    robot.text_to_frame(frame, 'speed = ' + str(speed), 260, 220)
    robot.text_to_frame(frame, color_line + " " + str(cross), 265, 300)

    robot.set_frame(frame, 40)  # отправка видео по WiFi
