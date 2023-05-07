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

lowblack = np.array([0, 70, 0])  # черный
upblack = np.array([180, 255, 45])

lowblue = np.array([84, 80, 34])  # синий
upblue = np.array([146, 255, 242])

loworange = np.array([5, 63, 78])  # оранжевый
uporange = np.array([39, 173, 186])

lowred = np.array([0, 85, 45])  # красный
upred = np.array([6, 225, 165])

lowgreen = np.array([64, 165, 51])  # зелёный
upgreen = np.array([81, 255, 221])

# координаты областей интереса

x_line_dat = [0, 220, 420, 640]  # координаты для датчиков линии
y_line_dat = [240, 280, 240, 280]

x_cross = [280, 360]  # координаты для датчика перекрёстка (оранжевой или синей линии)
y_cross = [320, 360]

x_cube = [120, 520]  # координаты для датчика кубиков (знаков)
y_cube = [200, 360]

# различные переменные для ПД

e_old = 0   # значение предыдущей ошибки для подсчёта дифференциальной составляющей
e_old_cube = 0  # значение предыдущей ошибки для подсчёта дифференциальной составляющей для регулятора для знаков
kp = 0.25   # коэффициент пропорциональной составляющей
kd = 0.2    # коэффициент дифференциальной составляющей
u = 0       # управляющее воздействие
e = 0       # ошибка (отклонение)
dat1, dat2 = 0, 0  # показания датчиков линии

cross = 0  # счётчик перекрёсков
color_line = 'none'  # цвет перекрёстка (оранжевый или синий) используется для определения направления движения
cube_color = 'none'  # цвет текущего знака

cub_pos = 0  # индекс для списка знаков
red_pos_x = 0  # позиция красного знака по х
red_pos_y = 0  # позиция красного знака по у
green_pos_x = 0  # позиция зелёного знака по х
green_pos_y = 0  # позиция зелёного знака по у
b_g = 200  # коэффициент для подсчёта отклонения от траектроии подъезда к зелёному знаку
b_r = 200  # коэффициент для подсчёта отклонения от траектроии подъезда к красному знаку

time_finish = 0  # время для финишной зоны засечённое с помощью функции search_cross()

state = 1  # переменная состояния
stop_flag = False

search_cross_time = time.time()  # таймеры
cross_time = time.time()
finish_tim = time.time()
cube_red_exist = time.time()
cube_green_exist = time.time()
cube_exist_tim = time.time()
stop_timer = time.time()
timer_sp = time.time()

flag_start = False  # флаги
flag_l = False
flag_wg = True
flag_wr = True
flag_cube_exist = False
flag_sort = True

speed = 40  # скорость
degree = 0  # угол поворота сервопривода

time_list = [0, 0, 0, 0]  # список времени зон получаемых из функции search_cross()
cub_col_list = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]  # список кубов по зонам
cub_num_list = [0, 0, 0, 0]  # список показывает сколько кубиков в каждой зоне
time_cub_list = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]  # список кубов по расположению в зоне (расчет по времени)


def wait_for_key():
    tx = '999999999999999999999$'  # сообщение об ожидании кнопки
    ii = '0'  # сообщение с микроконтроллера
    while ii == '0':  # пока сообщение равно "0"
        port.write(tx.encode("utf-8"))  # отправить сообщение об ожидании кнопки
        if port.in_waiting > 0:  # если что-то пришло
            ii = ""  # очищаем сообщение
            t = time.time()  # засекаем время
            while 1:  # всегда
                a = str(port.read(), "utf-8")  # получаем символ из сообщения
                if a != '$':  # если символ не стоп-символ
                    ii += a  # прибавить символ к сообщению
                else:  # иначе(пришёл стоп-символ)
                    break  # прекращаем читать сообщение
                if t + 0.02 < time.time():  # если вышел таймаут
                    break  # прекращаем читать сообщение


def black_search_left(d1):
    xm, ym, wm, hm = 0, 0, 0, 0
    dat = cv2.GaussianBlur(d1, (5, 5), cv2.BORDER_DEFAULT)  # размытие фрагмента изображения
    hsv = cv2.cvtColor(dat, cv2.COLOR_BGR2HSV)  # перевод в цветовую модель HSV
    mask = cv2.inRange(hsv, lowblack, upblack)  # создание маски
    blur = cv2.blur(mask, (5, 5))  # размытие маски

    imd1, contours, hod1 = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  # функция поиска контуров

    dat = 0
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)  # координаты текущего контура
        area = cv2.contourArea(contour)  # площадь текущего контура
        if area > 250:  # фильтр по минимальной площади контура
            if dat < w + x:  # поиск максимального по площади контура
                dat = (w + x)  # перезапись показаний датчиков
                xm, ym, wm, hm = x, y, w, h  # запись координат наибольшего контура

    return [xm + 1, ym + 1, xm + wm - 1, ym + hm - 1, int(dat / 220 * 100)]


def black_search_right(d1, w_dat):
    xm, ym, wm, hm = 0, 0, 0, 0
    dat = cv2.GaussianBlur(d1, (5, 5), cv2.BORDER_DEFAULT)  # размытие фрагмента изображения
    hsv = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)  # перевод в цветовую модель HSV
    mask = cv2.inRange(hsv, lowblack, upblack)  # создание маски
    blur = cv2.blur(mask, (5, 5))  # размытие маски

    imd1, contours, hod1 = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  # функция поиска контуров
    dat = 0
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)  # координаты текущего контура
        area = cv2.contourArea(contour)  # площадь текущего контура
        if area > 250:  # фильтр по минимальной площади контура
            if dat < w_dat - x:  # поиск максимального по площади контура
                dat = (w_dat - x)  # перезапись показаний датчиков
                xm, ym, wm, hm = x, y, w, h  # запись координат наибольшего контура

    return [xm + 1, ym + 1, xm + wm - 1, ym + hm - 1, int(dat / 220 * 100)]


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
    # отрисовка датчика знаков
    cv2.rectangle(frame, (x_cube[0], y_cube[0]), (x_cube[1], y_cube[1]), (60, 26, 28), 2)


def pd_regulator_cube(d1, d2, k_p=0.25, k_d=0.2):  # пропорционально-дифференциальный регулятор
    global e_old_cube, degree, color_line, u

    e = d2 - d1  # вычисяем отклонение
    if -10 < e < 10:  # если отклонение небольшое
        e = 0  # приравниваем к нулю
    u = int(e * k_p + (e - e_old_cube) * k_d)  # вычисляем управляющее воздействие по формуле ПД регулятора
    degree = u  # приравниваем угол к управляющему воздействию
    e_old_cube = e  # запоминаем предыдущую ошибку

    robot.text_to_frame(frame, e, 0, 40, (255, 255, 255), 1)


def pd_regulator(d1, d2, k_p=0.2, k_d=0.2):  # пропорционально-дифференциальный регулятор
    global e, e_old, degree, color_line, u

    e = d2 - d1  # вычисяем отклонение
    if -5 < e < 5:  # если отклонение небольшое
        e = 0  # приравниваем к нулю
    u = int(e * k_p + (e - e_old) * k_d)  # вычисляем управляющее воздействие по формуле ПД регулятора
    degree = u  # приравниваем угол к управляющему воздействию
    e_old = e  # запоминаем предыдущую ошибку

    if d1 == 0:  # если нет бортика поворачиваем в сторону где он должен быть
        degree = 35
    if d2 == 0:
        degree = -35

    if d1 == 0 and d2 == 0:  # если нет обоих бортиков поворачиваем в направлении движения
        if color_line == "orange":
            degree = -40
        elif color_line == "blue":
            degree = 40

    if d1 > 180 and d2 > 180:  # если оба датчика показывают больше 180
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
            if a1 > 530 and search_cross_time + 1.2 < time.time():
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
            if a1 > 530 and search_cross_time + 1.2< time.time():
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
    string = ",".join(lst)  # переводим сообщение в строку
    port.write(string.encode("utf-8"))  # отправляем сообщение


# def cube_y():  # функция поиска жёлтых кубиков
#     global cube_color, cross, sry, hy, t_coly, timer_sp, dop_per, cub
#     datk1 = frame[yk1:yk2, xk1:xk2]
#     xm, ym, wm, hm = 0, 0, 0, 0
#     hsv1 = cv2.cvtColor(datk1, cv2.COLOR_BGR2HSV)
#     maskd1 = cv2.inRange(hsv1, lowy, upy)
#     maskd2 = cv2.inRange(hsv1, lowr, upr)
#     mask2 = cv2.bitwise_not(maskd2)
#     mask = cv2.bitwise_and(mask2, maskd1)
#     imd1, contoursk, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
#
#     max = 0
#
#     if len(contoursk) != 0:
#         for contork1 in contoursk:
#             x, y, w, h = cv2.boundingRect(contork1)
#             a1 = cv2.contourArea(contork1)
#             if a1 > 300:
#                 t_coly = time.time()
#                 if y + h > max:
#                     max = y + h
#                     sry = x + w
#                     hy = y + h
#                     xm, ym, wm, hm = x, y, w, h
#                     timer_sp = time.time()
#
#     else:
#
#         if t_coly + 0.09 < time.time():
#             sry = 0
#             hy = 0
#             x1, y1, w1, h1 = 0, 0, 0, 0
#
#     cv2.rectangle(datk1, (x1, y1), (x1 + w1, y1 + h1), (0, 255, 255), 2)


def cube_r():  # функция поиска красных кубиков
    global cube_color, cross, red_pos_x, cub_pos, red_pos_y, cube_red_exist, timer_sp, flag_wr, \
        cross_time

    datk1 = frame[y_cube[0]:y_cube[1], x_cube[0]:x_cube[1]]
    hsv = cv2.cvtColor(datk1, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lowred, upred)
    blur = cv2.blur(mask, (5, 5))
    imd1, contours, hod1 = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    xm, ym, wm, hm = 0, 0, 0, 0

    if len(contours) != 0:
        max = 0
        for contor in contours:
            x, y, w, h = cv2.boundingRect(contor)
            area = cv2.contourArea(contor)
            if area > 450:
                cube_red_exist = time.time()
                if y + h > max:  # поиск наибольшего
                    max = y + h
                    xm, ym, wm, hm = x, y, w, h
                    timer_sp = time.time()
            else:
                if cube_red_exist + 0.15 < time.time():  # таймер для исчезновения кубика
                    xm, ym, wm, hm = 0, 0, 0, 0
        if ym > 80 and flag_wr and 0 < cross < 5:  # запись кубиков в список
            flag_wr = False
            cub_col_list[cross % 4][cub_pos % 3] = 'r'
            time_cub_list[cross % 4][cub_pos % 3] = round(time.time() - cross_time, 1)
            cub_pos += 1
        if ym < 30 and not flag_wr:
            flag_wr = True

    else:
        if cube_red_exist + 0.15 < time.time():
            xm, ym, wm, hm = 0, 0, 0, 0

    return [xm + 1, ym + 1, xm + wm - 1, ym + hm - 1]


def cube_g():  # функция поиска зеленых кубиков (за подробностями в cube_R())
    global green_pos_x, green_pos_y, cube_green_exist, timer_sp, flag_wg, cub_pos

    datg1 = frame[y_cube[0]:y_cube[1], x_cube[0]:x_cube[1]]
    hsv = cv2.cvtColor(datg1, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lowgreen, upgreen)
    blur = cv2.blur(mask, (5, 5))
    imd1, contoursk, hod1 = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max = 0

    xm, ym, wm, hm = 0, 0, 0, 0

    if len(contoursk) != 0:
        for contork1 in contoursk:
            x, y, w, h = cv2.boundingRect(contork1)
            area = cv2.contourArea(contork1)
            if area > 450:
                cube_green_exist = time.time()
                if y + h > max:
                    max = y + h
                    xm, ym, wm, hm = x, y, w, h
                    timer_sp = time.time()
            else:
                if cube_green_exist + 0.15 < time.time():
                    xm, ym, wm, hm = 0, 0, 0, 0
        if ym > 80 and flag_wg and 0 < cross < 5:
            cub_col_list[cross % 4][cub_pos % 3] = 'g'
            flag_wg = False
            time_cub_list[cross % 4][cub_pos % 3] = round(time.time() - cross_time, 1)
            cub_pos += 1
        if ym < 30 and not flag_wg:
            flag_wg = True
    else:
        if cube_green_exist + 0.15 < time.time():
            xm, ym, wm, hm = 0, 0, 0, 0

    return [xm + 1, ym + 1, xm + wm - 1, ym + hm - 1]


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


def max_min_index(list_i):
    max_i = 0
    min_i = 0
    for i in range(len(list_i)):
        if list_i[max_i] < list_i[i]:
            max_i = i
        if list_i[min_i] > list_i[i]:
            min_i = i
    return max_i, min_i


def cub_list_sort():
    global time_list, cub_num_list, cub_col_list, time_cub_list
    for i in range(4):
        for j in range(3):
            if cub_col_list[i][j] != 0:
                cub_num_list[i] += 1

    for i in range(4):
        if cub_num_list[i] == 2:
            max_i, min_i = max_min_index(time_cub_list[i])
            time_cub_list[i][max_i], time_cub_list[i][2] = time_cub_list[i][2], time_cub_list[i][max_i]
            cub_col_list[i][max_i], cub_col_list[i][2] = cub_col_list[i][2], cub_col_list[i][max_i]
            max_i, min_i = max_min_index(time_cub_list[i])
            if min_i != 1:
                time_cub_list[i][min_i], time_cub_list[i][1] = time_cub_list[i][1], time_cub_list[i][min_i]
                cub_col_list[i][min_i], cub_col_list[i][1] = cub_col_list[i][1], cub_col_list[i][min_i]

        elif cub_num_list[i] == 1:
            max_i, min_i = max_min_index(time_cub_list[i])
            if time_cub_list[i][max_i] <= 0.7:
                time_cub_list[i][max_i], time_cub_list[i][0] = time_cub_list[i][0], time_cub_list[i][max_i]
                cub_col_list[i][max_i], cub_col_list[i][0] = cub_col_list[i][0], cub_col_list[i][max_i]
            elif time_cub_list[i][max_i] <= 1.5:
                time_cub_list[i][max_i], time_cub_list[i][1] = time_cub_list[i][1], time_cub_list[i][max_i]
                cub_col_list[i][max_i], cub_col_list[i][1] = cub_col_list[i][1], cub_col_list[i][max_i]
            else:
                time_cub_list[i][max_i], time_cub_list[i][2] = time_cub_list[i][2], time_cub_list[i][max_i]
                cub_col_list[i][max_i], cub_col_list[i][2] = cub_col_list[i][2], cub_col_list[i][max_i]


wait_for_key()

green_pos = None
red_pos = None

while 1:
    frame = robot.get_frame(wait_new_frame=1)

    if state == 1:  # езда
        red, green, blue = 0, 0, 0  # переменные хранящие состояние RGB светодиода
        if search_cross_time + 0.5 > time.time():  # если с момента засечения перекрёстка прошло менее 0.5 секунд
            if color_line == 'orange':  # если цвет перекрёстка оранжевый зажечь оранжевый
                red = 60
                green = 50
            if color_line == 'blue':  # если цвет перекрёстка синий зажечь синий
                blue = 80
                green = 20
        green_pos = cube_g()
        red_pos = cube_r()
        green_pos_x = green_pos[0] - 1
        green_pos_y = green_pos[3]
        red_pos_x = red_pos[2] + 1
        red_pos_y = red_pos[3]

        if green_pos_x == 0 and red_pos_x == 0:  # если нет кубиков
            detect_line_pro()  # ищем бортики
            # if cube_exist_tim + 0.3 < time.time():
            #     x_line_dat[2] = 420
            #     x_line_dat[1] = 220
            pd_regulator(dat1, dat2, kp, kd)
            if cube_green_exist + 0.4 > time.time() and color_line == 'orange':
                degree = -50
            if cube_red_exist + 0.4 > time.time() and color_line == 'blue':
                degree = 50

        elif green_pos_x > 0 and green_pos_y > red_pos_y:  # если есть зелёный куб и он ближе
            cube_color = "Green"
            cube_exist_tim = time.time()
            if color_line == 'blue':
                b_g = 150
            elif color_line == 'orange':
                b_g = 180
            else:
                b_g = 160
            pd_regulator_cube(green_pos_x, (b_g + green_pos_y * 1.23))  # ошибка высчитывается исходя из перспективы
            red, green, blue = 0, 100, 20

        else:  # если есть красный
            cube_color = "Red"
            cube_exist_tim = time.time()
            if color_line == 'blue':
                b_r = 230
            elif color_line == 'orange':
                b_r = 220
            else:
                b_r = 240
            pd_regulator_cube(red_pos_x, (b_r - red_pos_y * 1.23))  # ошибка высчитывается исходя из перспективы
            red, green, blue = 100, 0, 0

        search_cross()

    if state == 2:  # stop
        if stop_timer + 0.1 > time.time():
            speed = -100
        else:
            state = 3

    if state == 3:
        degree = 0
        speed = 0

    if cross == 12 and not stop_flag:  # если проехали 12 перекрёстков и флаг опущен
        stop_flag = True  # поднимаем флаг
        finish_tim = time.time()  # засекаем время
        cross = 13

    if finish_tim + time_finish < time.time() and stop_flag:
        # если с момента поднятия флага прошло время необходимое для проезда в центр зоны
        state = 2  # переходим в состояние "стоп"
        stop_timer = time.time()
        stop_flag = False

    fps1 += 1
    if time.time() > fps_time + 1:
        fps_time = time.time()
        fps = fps1
        fps1 = 0

    if cross == 6 and flag_sort:
        cub_list_sort()
        flag_sort = False

    draw_contour_line()

    print_message(speed, degree, red, green, blue)

    cv2.rectangle(frame, (0, 360), (640, 480), (0, 0, 0), -1)
    # cv2.rectangle(frame, (0, 0), (640, 120), (0, 0, 0), -1)
    cv2.line(frame, (x_cube[0] + b_r, y_cube[0]), (x_cube[0], y_cube[1]), (10, 20, 140), 2)
    cv2.line(frame, (x_cube[0] + b_g, y_cube[0]), (x_cube[1], y_cube[1]), (10, 140, 20), 2)

    robot.text_to_frame(frame, 'fps = ' + str(fps), 3, 470, (255, 255, 255), 1)  # телеметрия
    robot.text_to_frame(frame, color_line + " " + str(cross), 260, 400, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'degree:' + str(degree), 390, 400, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'speed:' + str(speed), 390, 380, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'red_pos_x:' + str(red_pos_x), 3, 380, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'red_pos_y:' + str(red_pos_y), 3, 400, (255, 255, 255), 1)
    robot.text_to_frame(frame, str(cub_col_list), 3, 420, (255, 255, 255), 1)
    robot.text_to_frame(frame, str(time_cub_list), 3, 440, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'timelist' + str(time_list), 130, 470, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'cubnumlist: ' + str(cub_num_list), 3, 20, (255, 255, 255), 1)
    robot.text_to_frame(frame, dat1, 0, 290, (255, 255, 255), 1)
    robot.text_to_frame(frame, dat2, 600, 290, (255, 255, 255), 1)
    cv2.rectangle(frame[y_cube[0]:y_cube[1], x_cube[0]:x_cube[1]], (red_pos[0], red_pos[1]),
                  (red_pos[2], red_pos[3]),
                  (50, 50, 250), -1)
    cv2.rectangle(frame[y_cube[0]:y_cube[1], x_cube[0]:x_cube[1]], (green_pos[0], green_pos[1]),
                  (green_pos[2], green_pos[3]), (50, 250, 50), -1)
    robot.set_frame(frame, 40)
