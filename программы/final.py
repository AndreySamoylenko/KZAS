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

lowblack = np.array([59, 111, 8])
upblack = np.array([116, 256, 67])

lowblue = np.array([86, 142, 26])
upblue = np.array([116, 256, 256])

loworange = np.array([6, 63, 78])
uporange = np.array([39, 173, 186])

lowred = np.array([0, 120, 70])
upred = np.array([7, 225, 125])

lowred1 = np.array([176, 128, 47])
upred1 = np.array([180, 210, 256])

lowgreen = np.array([36, 114, 60])
upgreen = np.array([81, 256, 216])

# координаты для отрисовки всех датчиков

x_line_dat = [0, 220, 420, 640]  # координаты для датчиков линии
y_line_dat = [240, 270, 240, 270]

x_cross = [280, 360]  # координаты для датчика перекрёстка (оранжевой или синей линии)
y_cross = [310, 340]

x_cube = [60, 580]  # координаты для датчика кубиков (знаков)
y_cube = [130, 290]

# различные переменные для ПД

u = 0  # управляющее воздействие
e = 0  # ошибка (отклонение)
e_old = 0
dat1, dat2 = 0, 0  # показания датчиков линии
kpnc = 0.25
kdnc = 1

l1, l2 = [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]  # списки для отрисовки контуров черной линии

cross = 0  # счётчик перекрёсков
color_line = 'none'  # цвет перекрёстка (оранжевый или синий) используется для определения направления движения
cube_color = 'none'

cub_pos = 0
red_pos_x = 0
red_pos_y = 0
green_pos_x = 0
green_pos_y = 0
b_g = 0
b_r = 0

time_finish = 0  # время для финишной зоны засечённое с помощью функции search_cross()

state = 1  # переменные состояния
stop_flag = False

search_cross_time = time.time()  # таймеры
cross_time = time.time()
finish_tim = time.time()
# x_road_tim = time.time()
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

message = ""  # сообщение формируемое функцией print_message()

speed = 35  # скорость
degree = 0  # угол поворота сервопривода

time_list = [0, 0, 0, 0]  # список времени зон получаемых из функции search_cross()
cub_col_list = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]  # список кубов по зонам
cub_num_list = [0, 0, 0, 0]  # список показывает сколько кубиков в каждой зоне
time_cub_list = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]  # список кубов по расположению в зоне (расчет по времени)


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


def black_search_left(d1):
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
        if area > 400:
            if max1 < h * w:
                max1 = h * w
                dat = (w + x)
                xm, ym, wm, hm = x, y, w, h

    return [xm + 1, ym + 1, xm + wm - 1, ym + hm - 1, dat]


def black_search_right(d1, w_dat):
    xm, ym, wm, hm = 0, 0, 0, 0
    dat = cv2.GaussianBlur(d1, (5, 5), cv2.BORDER_DEFAULT)
    hsv = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lowblack, upblack)  #
    blur = cv2.blur(mask, (5, 5))

    im, contours, ho = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  #
    max1 = 0
    dat = 0
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area > 400:
            if max1 < h * w:
                max1 = h * w
                dat = (w_dat - x)
                xm, ym, wm, hm = x, y, w, h

    return [xm + 1, ym + 1, xm + wm - 1, ym + hm - 1, dat]


def detect_line_pro():
    global x_line_dat, y_line_dat, dat1, dat2, l1, l2
    # забираем часть экрана для датчиков

    l1 = black_search_left(frame[y_line_dat[0]:y_line_dat[1], x_line_dat[0]:x_line_dat[1]])
    l2 = black_search_right(frame[y_line_dat[2]:y_line_dat[3], x_line_dat[2]:x_line_dat[3]], 200)
    # высчитываем среднее исходя из показаний датчиков
    dat1 = (l1[4])
    dat2 = (l2[4])


def draw_contour_line():
    cv2.rectangle(frame, (x_line_dat[0], y_line_dat[0]), (x_line_dat[1], y_line_dat[1]), (0, 25, 200), 2)
    cv2.rectangle(frame, (x_line_dat[2], y_line_dat[2]), (x_line_dat[3], y_line_dat[3]), (0, 25, 200), 2)

    cv2.rectangle(frame, (x_cross[0], y_cross[0]), (x_cross[1], y_cross[1]), (0, 205, 200), 2)

    cv2.rectangle(frame[y_line_dat[0]:y_line_dat[1], x_line_dat[0]:x_line_dat[1]], (l1[0], l1[1]), (l1[2], l1[3]),
                  (10, 245, 0), 2)
    cv2.rectangle(frame[y_line_dat[2]:y_line_dat[3], x_line_dat[2]:x_line_dat[3]], (l2[0], l2[1]), (l2[2], l2[3]),
                  (10, 245, 0), 2)

    cv2.rectangle(frame, (x_cube[0] + 20, y_cube[0]), (x_cube[1] + 20, y_cube[1]), (0, 50, 20), 2)
    cv2.rectangle(frame, (x_cube[0] - 20, y_cube[0]), (x_cube[1] - 20, y_cube[1]), (0, 20, 50), 2)


def pd_regulator(d1, d2, kp=0.25, kd=1):  # пропорционально-дифференциальный регулятор
    global e, e_old, degree, color_line, u, cube_exist_tim

    e = d2 - d1
    if -5 < e < 5:
        e = 0
    u = int(e * kp + (e - e_old) * kd)
    degree = u
    e_old = e  # до сюда обычный пропорционально-дифференциальный регулятор

    if d1 == 0:
        if cube_exist_tim + 0.3 > time.time():
            degree = 30
        else:
            degree = 60

    if d2 == 0:
        if cube_exist_tim + 0.3 > time.time():
            degree = -38
        else:
            degree = -60

    if d1 == 0 and d2 == 0:
        if color_line == "orange":
            if cube_exist_tim + 0.25 > time.time():
                degree = -40
            else:
                degree = -60
        elif color_line == "blue":
            if cube_exist_tim + 0.25 > time.time():
                degree = 35
            else:
                degree = 60


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
            area = cv2.contourArea(contour)
            if area > 200 and search_cross_time + 0.9 < time.time():
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
            if a1 > 200 and search_cross_time + 0.9 < time.time():
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

    datk1 = frame[y_cube[0]:y_cube[1], x_cube[0] + 20:x_cube[1] + 20]
    hsv = cv2.cvtColor(datk1, cv2.COLOR_BGR2HSV)
    # gblur = cv2.GaussianBlur(hsv, (5, 5), cv2.BORDER_DEFAULT)
    mask1 = cv2.inRange(hsv, lowred, upred)
    mask2 = cv2.inRange(hsv, lowred1, upred1)  # поиск по HSV
    mask = cv2.bitwise_or(mask1, mask2)
    blur = cv2.blur(mask, (5, 5))
    imd1, contours, hod1 = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    xm, ym, wm, hm = 0, 0, 0, 0
    max = 0

    if len(contours) != 0:
        for contor in contours:
            x, y, w, h = cv2.boundingRect(contor)
            area = cv2.contourArea(contor)
            if area > 450:
                cube_red_exist = time.time()
                if y + h > max:  # поиск наибольшего
                    max = y + h
                    red_pos_x = x + w
                    red_pos_x = y + h
                    xm, ym, wm, hm = x, y, w, h
                    timer_sp = time.time()
            else:
                if cube_red_exist + 0.1 < time.time():  # таймер для исчезновения кубика
                    red_pos_x = 0
                    red_pos_y = 0
                    xm, ym, wm, hm = 0, 0, 0, 0
        if ym > 80 and flag_wr and 0 < cross < 5:  # запись кубиков в список
            flag_wr = False
            cub_col_list[cross % 4][cub_pos % 3] = 'r'
            time_cub_list[cross % 4][cub_pos % 3] = round(time.time() - cross_time, 1)
            cub_pos += 1
        if ym < 30 and not flag_wr:
            flag_wr = True

    else:
        if cube_red_exist + 0.1 < time.time():
            red_pos_x = 0
            red_pos_y = 0
            xm, ym, wm, hm = 0, 0, 0, 0

    return [xm + 1, ym + 1, xm + wm - 1, ym + hm - 1]


def cube_g():  # функция поиска зеленых кубиков (за подробностями в cube_R())
    global green_pos_x, green_pos_y, cube_green_exist, timer_sp, flag_wg, cub_pos

    datg1 = frame[y_cube[0]:y_cube[1], x_cube[0] - 20:x_cube[1] - 20]
    hsv1 = cv2.cvtColor(datg1, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv1, lowgreen, upgreen)
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
                    green_pos_x = x
                    green_pos_y = y + h
                    xm, ym, wm, hm = x, y, w, h
                    timer_sp = time.time()
            else:
                if cube_green_exist + 0.1 < time.time():
                    green_pos_x = 0
                    green_pos_y = 0
                    xm, ym, wm, hm = 0, 0, 0, 0
        if ym > 80 and flag_wg and 0 < cross < 5:
            cub_col_list[cross % 4][cub_pos % 3] = 'g'
            flag_wg = False
            time_cub_list[cross % 4][cub_pos % 3] = round(time.time() - cross_time, 1)
            cub_pos += 1
        if ym < 30 and not flag_wg:
            flag_wg = True
    else:
        if cube_green_exist + 0.1 < time.time():
            green_pos_x = 0
            green_pos_y = 0
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
    # state_changer()
    frame = robot.get_frame(wait_new_frame=1)

    if state == 1:  # езда
        green_pos = cube_g()
        red_pos = cube_r()
        green_pos_x = green_pos[0] - 1
        green_pos_y = green_pos[3]
        red_pos_x = red_pos[2] + 1
        red_pos_y = red_pos[3]

        if green_pos_x == 0 and red_pos_x == 0:  # если нет кубиков
            detect_line_pro()  # ищем бортики
            kpnc = 0.25
            if 0.2 < time.time() - cube_exist_tim < 0.6:  # после кубика поворот в направлении движения
                kpnc = 0.15
                if cube_color == "Green":
                    if color_line == 'orange':
                        degree = -45
                    if color_line == 'blue':
                        if dat1 > 80:
                            degree = 50
                        else:
                            degree = 30

                elif cube_color == "Red":
                    if color_line == 'blue':
                        degree = 45
                    if color_line == 'orange':
                        if dat2 > 80:
                            degree = -50
                        else:
                            degree = -30
                flag_cube_exist = True
            elif flag_cube_exist:
                cube_color = "none"
                flag_cube_exist = False
            pd_regulator(dat1, dat2, kpnc, kdnc)
            if dat1 > 160 and dat2 > 160:
                if color_line == "orange":
                    if cube_exist_tim + 0.25 > time.time():
                        degree = -60
                    else:
                        degree = -40
                elif color_line == "blue":
                    if cube_exist_tim + 0.25 > time.time():
                        degree = 60
                    else:
                        degree = 40

        elif green_pos_x != 0 and green_pos_y > red_pos_y:  # если есть зелёный куб и он ближе
            if time.time() - cube_red_exist < 0.2:
                kpg = 0.2
            else:
                kpg = 0.35
            cube_color = "Green"
            cube_exist_tim = time.time()
            # e = (205 + green_pos_y * 1.3) - green_pos_x  # ошибка высчитывается исходя из перспективы
            if color_line == 'blue':
                b_g = 150
            elif color_line == 'orange':
                b_g = 190
            else:
                b_g = 170
            pd_regulator(green_pos_x, (b_g + green_pos_y * 1.67), kpg, 1)

        else:  # если есть красный
            if time.time() - cube_green_exist < 0.2:
                kpr = 0.2
            else:
                kpr = 0.35
            cube_color = "Red"
            cube_exist_tim = time.time()
            # e = (275 - red_pos_y * 1.3) - red_pos_x      # ошибка высчитывается исходя из перспективы
            if color_line == 'blue':
                b_r = 330
            elif color_line == 'orange':
                b_r = 350

            pd_regulator(red_pos_x, (b_r - red_pos_y * 1.67), kpr, 1)

        search_cross()

    if state == 2:  # stop
        if stop_timer + 0.1 > time.time():
            speed = -speed
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
    port.write(message.encode("utf-8"))  # отправка сообщения

    cv2.rectangle(frame, (0, 340), (640, 480), (0, 0, 0), -1)
    cv2.rectangle(frame, (0, 0), (640, 120), (0, 0, 0), -1)
    cv2.drawline(frame, (x_cube[0],170+y_cube[0]))
    robot.text_to_frame(frame, 'fps = ' + str(fps), 3, 470, (255, 255, 255), 1)  # телеметрия
    robot.text_to_frame(frame, color_line + " " + str(cross), 265, 400, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'degree:' + str(degree), 250, 360, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'red_pos_x:' + str(red_pos_x), 3, 380, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'red_pos_y:' + str(red_pos_y), 3, 400, (255, 255, 255), 1)
    robot.text_to_frame(frame, str(cub_col_list), 3, 420, (255, 255, 255), 1)
    robot.text_to_frame(frame, str(time_cub_list), 3, 440, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'speed:' + str(speed), 260, 380, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'timelist' + str(time_list), 130, 470, (255, 255, 255), 1)
    robot.text_to_frame(frame, 'cubnumlist: ' + str(cub_num_list), 3, 20, (255, 255, 255), 1)
    robot.text_to_frame(frame, dat1, 0, 360, (255, 255, 255), 1)
    robot.text_to_frame(frame, dat2, 600, 360, (255, 255, 255), 1)
    cv2.rectangle(frame[y_cube[0]:y_cube[1], x_cube[0] + 20:x_cube[1] + 20], (red_pos[0], red_pos[1]),
                  (red_pos[2], red_pos[3]),
                  (50, 50, 250), -1)
    cv2.rectangle(frame[y_cube[0]:y_cube[1], x_cube[0] - 20:x_cube[1] - 20], (green_pos[0], green_pos[1]),
                  (green_pos[2], green_pos[3]), (50, 250, 50), -1)
    robot.set_frame(frame, 40)
