from pyb import delay, Pin, ADC, Timer, UART
import pyb  # импортируем внутреннюю библиотеку пайборда

# настраиваем UART(протокол общения между pyboard и raspberry
uart = UART(2, 115200, stop=1)  # пин для UART
inn = ''

# задаём пины для работы
servo = pyb.Servo(1)  # пин для сервопривода
servo.angle(0)  # поворот сервы в 0 градусов

RED = pyb.LED(1)
GREEN = pyb.LED(2)  # пины для встроенных светодиодов пайборда
YELLOW = pyb.LED(3)
BLUE = pyb.LED(4)

p_in = Pin('X10', Pin.IN, Pin.PULL_UP)  # пин кнопки

Ma = Pin('Y10', Pin.OUT_PP)  # пины для работы с драйвером
Mb = Pin('Y9', Pin.OUT_PP)

Sp = Pin('X8')  # пин скорости является ШИМ пином
tim = Timer(14, freq=10000)
ch = tim.channel(1, Timer.PWM, pin=Sp)

# пины для RGB светодиода
Spr = Pin('Y8')
tim = Timer(1, freq=2000)
r = tim.channel(3, Timer.PWM, pin=Spr)
Spg = Pin('Y6')
tim = Timer(1, freq=2000)
g = tim.channel(1, Timer.PWM, pin=Spg)
Spb = Pin('Y7')
tim = Timer(1, freq=2000)
b = tim.channel(2, Timer.PWM, pin=Spb)


def RGB_LED(red, green, blue):
    r.pulse_width_percent(red)
    g.pulse_width_percent(green)
    b.pulse_width_percent(blue)


def motor(sped):  # функция упрощённого управления мотором
    global Ma, Mb, ch
    if sped > 0:
        Ma.low()
        Mb.high()
        ch.pulse_width_percent(sped)
    else:
        Mb.low()
        Ma.high()
        ch.pulse_width_percent(-sped)


def decrypt(strin):
    total = [0, 0, 0, 0, 0]
    lst = strin.split(',')
    for i in range(5):
        total[i] = int(lst[i]) - 200
    return total


message = ""
flag_start = False
YELLOW.off()
a = 0

for i in range(100):
    RGB_LED(i, 100 - i, 0)
    delay(10)

for i in range(100):
    RGB_LED(100 - i, 0, i)
    delay(10)

for i in range(100):
    RGB_LED(0, i, 100 - i)
    delay(10)

while 1:
    if uart.any():  # обработка сообщений
        YELLOW.on()
        a = chr(uart.readchar())  # посимвольное чтение сообщение
        if a != '$':  # если не обгаружен стоп-символ
            inn += a  # добавляем символ к сообщению
            if len(inn) > 20:  # отсечение по длине (если сообщение длиннее чем должно быть то мы его игнорируем)
                inn = ""
        else:
            if not flag_start:  # сообщение "99999999999999999999" говорит о том что распберри запустилась
                if inn == "99999999999999999999":
                    flag_start = True
                    BLUE.on()  # как индикация включается синий светодиод
                    RGB_LED(99, 99, 99)  # и RGB светодиод горит белым
            else:
                message = str(p_in.value()) + '$'  # формируем сообщение с показанием кнопки
                try:
                    if inn != '99999999999999999999' and len(inn) == 20:  # чтение сообщения и езда
                        print(inn)
                        spisok = decrypt(inn)  # читаем сообщение и разбиваем его на список
                        speed = int(spisok[0])  # из списка с сообщением достаём скорость и угол поворота
                        rul = int(spisok[1])
                        rul = -rul  # угол поворота инвертируем только если необходимо
                        #             (это обусловлено конструкцией)
                        if rul > 60:  # ограничение угла поворота
                            rul = 60
                        if rul < -60:
                            rul = -60
                        RGB_LED(spisok[2], spisok[3], spisok[4])  # из списка с сообщением достаём цвет светодиода
                        motor(speed)  # запускаем мотор со скоростью из сообщения
                        servo.angle(rul - 7)  # поворачиваем сервопривод на заданный угол
                        # rul - 7 нужно для того, чтобы угол сервы совпадал с 0 переменной rul
                    else:
                        print(inn)
                except ValueError:
                    print("err")

            inn = ""
            uart.write(message)  # отсылаем показания кнопки
    else:
        YELLOW.off()
