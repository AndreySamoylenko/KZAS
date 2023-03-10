# то что помечемо комментарием - фишки

def pd_regulator(dat1,dat2):              # пропорционально-дифференциальный регулятор
    global e, e_old, deg, color_per, u

    e = dat2 - dat1
    if -5 < e < 5:                        # игнор маленькой ошибки
        e = 0
    u = int(e * kp + (e - e_old) * kd)
    deg = u
    e_old = e

    if dat1 == 0:                          # потеря бортика датчиком происходит при повороте
        deg = 35                           # следовательно поворачиваем в сторону отсутствующего датчика
    if dat2 == 0:                          # ВНИМАНИЕ регулятор не может отработать такую ситуацию
        deg = -35                          # второй датчик шумит и поворот не максимальный

    if dat1 == 0 and dat2 == 0:            # при потере обоих бортиков поворот в направлении движения
        if color_per == "orange":
            deg = -30
        elif color_per == "blue":
            deg = 30

    if deg > 90:                            # ограничение по углу
        deg = 90
    if deg < -90:
        deg = -90


# ниже будут горизонтальные датчики


def black_poisk_l(d1):
    xm, ym, wm, hm =0,0,0,0
    dat=cv2.GaussianBlur(d1,(5,5),cv2.BORDER_DEFAULT)
    hsv = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)
    blur = cv2.blur(hsv,(5,5))
    mask = cv2.inRange(blur, lowblack, upblack)  # HSV

    imd1, contours, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  #
    max1 = 0
    dat = 0
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area > 500:
            if max1 < h * w:
                max1 = h * w
                dat = h * (w+x)             # высчитываем среднее по площади прямоугольника от легого края экрана
                xm, ym, wm, hm = x, y, w, h                                             # до правого края контура

    return [xm + 1,ym + 1,xm + wm - 1, ym + hm - 1,dat] # список для отрисовки прямоугольника и среднего значения

def black_poisk_r(d1,w_dat):
    xm, ym, wm, hm =0,0,0,0
    dat=cv2.GaussianBlur(d1,(5,5),cv2.BORDER_DEFAULT)
    hsv = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)
    blur = cv2.blur(hsv,(5,5))
    mask = cv2.inRange(blur, lowblack, upblack)  #

    imd1, contours, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max1 = 0
    dat = 0
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)             # тут всё тоже самоё
        if area > 500:
            if max1 < h * w:
                max1 = h * w
                dat = h * (w_dat-x)                 # но площадь прямоугольника рассчитывается от правого края экрана
                xm, ym, wm, hm = x, y, w, h                                                  # до левого края контура
    return [xm + 1,ym + 1,xm + wm - 1, ym + hm - 1,dat]



def bortik_pro():
    global x, y, dat1, dat2

    d1 = frame[y[0]:y[1], x[0]:x[1]]
    d2 = frame[y[2]:y[3], x[2]:x[3]]
    d3 = frame[y[4]:y[5], x[4]:x[5]]
    d4 = frame[y[6]:y[7], x[6]:x[7]]  # забираем часть экрана длядатчиков

    dat1 = (black_poisk_l(d1)[4] + black_poisk_l(d2)[4]) // 100  # высчитываем среднее исходя из показаний датчиков
    dat2 = (black_poisk_r(d3, 125)[4] + black_poisk_r(d4, 250)[4]) // 100


# ниже будут кубики

def cube_R():# функция поиска красных кубиков
    global xk1, yk1, xk2, yk2, lowr, upr, color_Final, perek, i, srr,speed,hr,t_colr,timer_sp,cube,flag,timerCVET,\
        flag_wr,tim_per
    datk1 = frame[yk1:yk2, xk1:xk2]

    hsv1 = cv2.cvtColor(datk1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1, lowr, upr)
    maskd2=cv2.inRange(hsv1,lowr1,upr1)             # поиск по HSV
    mask=cv2.bitwise_or(maskd1,maskd2)              # объединение масок
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


                if y+h>max:      # поиск ближнего
                    max=y+h
                    srr = x+w
                    hr = y + h
                    x1, y1, w1, h1 =  x, y, w, h
                    timer_sp=time.time()
            else:
                if t_colr + 0.05 < time.time():    # таймер для исчезновения кубика
                    srr = 0
                    hr = 0
                    x1, y1, w1, h1 = 0, 0, 0, 0
    else:

        if t_colr+0.05< time.time():               # таймер для исчезновения кубика
            srr = 0
            hr = 0
            x1, y1, w1, h1 = 0, 0, 0, 0

    cv2.rectangle(datk1, (x1, y1), (x1 + w1, y1 + h1), (0, 0, 255), 2)
    cv2.rectangle(frame, (xk1, yk1), (xk2, yk2), (0, 200,200), 2)