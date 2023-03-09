from pyb import delay, Pin, ADC, Timer, UART
import pyb  # импортируем внутреннюю библиотеку пайборда

# задаём пины для работы
uart = UART(2, 115200, stop=1)  # пин для UART
inn = ''

servo = pyb.Servo(1)  # пин для сервопривода
servo.angle(0)  # поворот сервы в 0 градусов

RED = pyb.LED(1)
GREEN = pyb.LED(2)  # пины для встроенных светодиодов пайборда
YELLOW = pyb.LED(3)
BLUE = pyb.LED(4)

p_in = Pin('X10', Pin.IN, Pin.PULL_UP)  # кнопка

Ma = Pin('Y10', Pin.OUT_PP)
Mb = Pin('Y9', Pin.OUT_PP)
Sp = Pin('X8')
tim = Timer(14, freq=10000)
ch = tim.channel(1, Timer.PWM, pin=Sp)  # пины для работы с драйвером
Spr = Pin('Y8')
tim = Timer(1, freq=2000)
r = tim.channel(3, Timer.PWM, pin=Spr)
Spg = Pin('Y6')
tim = Timer(1, freq=2000)
g = tim.channel(1, Timer.PWM, pin=Spg)
Spb = Pin('Y7')
tim = Timer(1, freq=2000)
b = tim.channel(2, Timer.PWM, pin=Spb)


def RGB_LED(red, gren, blu):
    r.pulse_width_percent(red)
    g.pulse_width_percent(gren)
    b.pulse_width_percent(blu)


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


def stop():
    Mb.high()
    Ma.high()


def decrypt(strin):
    total = [0, 0, 0, 0, 0]
    lst = strin.split(',')
    for i in range(5):
        total[i] = int(lst[i]) - 200
    return total


message = ""
flag_start = True
YELLOW.off()
a = 0

for i in range(100):
    RGB_LED(i, 100 - i, 0)
    delay(15)
for i in range(100):
    RGB_LED(100 - i, 0, i)
    delay(15)
for i in range(100):
    RGB_LED(0, i, 100 - i)
    delay(15)

while 1:
    if uart.any():  # обработка сообщений
        YELLOW.on()
        a = chr(uart.readchar())
        if a != '$':
            inn += a
            if len(inn) > 21:
                inn = ""
        else:
            if flag_start:  # сообщение "99999999999999999999" говорит о том что распберри запустилась и готова к работе
                if inn == "99999999999999999999":
                    flag_start = False
                    BLUE.on()  # как индикация включается синий светодиод
                    RGB_LED(99, 99, 99)
            else:
                message = str(p_in.value()) + '$'  # отсылаем показания кнопки
                try:
                    if inn != '99999999999999999999' and len(inn) == 20:  # чтение сообщения и езда
                        print(inn)
                        spisok = decrypt(inn)
                        print(spisok)
                        rul = int(spisok[1])
                        rul = -rul
                        speed = int(spisok[0])
                        print(speed, rul)
                        if rul > 40:
                            rul = 40
                        if rul < -40:
                            rul = -40
                        RGB_LED(spisok[2], spisok[3], spisok[4])
                        motor(speed)
                        servo.angle(rul + 4)  # rul-4 нужно для того, чтобы угол сервы совпадал с 0 переменной rul
                    else:
                        print(inn)
                except ValueError:
                    print("err")

            inn = ""
            uart.write(message)  # отправка кнопки
