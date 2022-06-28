# main.py -- put your code here!
# main.py -- put your code here!
from pyb import delay, Pin, ADC, Timer, UART
import pyb                                  # импортируем внутреннюю библиотеку пайборда


                                            # задаём пины для работы
uart = UART(6, 115200, stop=1)              # пин для UART
inn = ''

servo = pyb.Servo(1)                        # пин для сервопривода
servo.angle(0)                              # поворот сервы в 0 градусов

RED= pyb.LED(1)
GREEN=pyb.LED(2)                            # пины для встроенных светодиодов пайборда
YELLOW = pyb.LED(3)
BLUE=pyb.LED(4)

PIC= Pin('X7', Pin.OUT_PP)             # пишалка
p_in = Pin('X8', Pin.IN, Pin.PULL_UP)  # кнопка


Ma = Pin('Y7', Pin.OUT_PP)
Mb = Pin('Y8', Pin.OUT_PP)
Sp = Pin('X9')
tim = Timer(4, freq = 10000)
ch = tim.channel(1, Timer.PWM, pin = Sp)# пины для работы с драйвером



def motor(speed):  # функция упрощённого управления мотором
    global Ma , Mb, ch
    if speed>0:
        Ma.low()
        Mb.high()
        ch.pulse_width_percent(speed)
    else:
        Mb.low()
        Ma.high()
        ch.pulse_width_percent(-speed)

message=""
flag_start=True
YELLOW.off()
a=0
while 1:
    if uart.any():          # обработка сообщений
        YELLOW.on()
        a = chr(uart.readchar())
        if a != '$':
            inn += a
            if len(inn) > 6:
                inn = ""

        else:
            if flag_start:          # сообщение "999999" говорит о том что распберри запустилась и готова к работе и
                                    # как индикация вклучается синий светодиод
                if inn == "999999":
                    flag_start = False
                    BLUE.on()
                    PIC.high()
                    delay(500)
                    PIC.low()
            else:
                message = str(1 - p_in.value()) + '$'   # отсылаем показания кнопки
                try:
                    if len(inn) == 6 and inn != '999999': # чтение сообщения и езда
                        rul = int(inn[:3]) - 200
                        speed = int(inn[3:]) - 200
                        print(speed, rul)
                        if rul>70:
                            rul=70
                        if rul<-60:
                            rul=-60
                        motor(speed)
                        servo.angle(rul)



                except ValueError:
                    print("err")

            print(inn)# отправка кнопки
            inn = ""
            uart.write(message)