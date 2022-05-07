# main.py -- put your code here!
# main.py -- put your code here!
from pyb import delay, Pin, ADC, Timer, UART
import pyb

uart = UART(6, 115200, stop=1)
inn = ''

servo = pyb.Servo(1)
servo.angle(0)

RED= pyb.LED(1)
GREEN=pyb.LED(2)
YELLOW = pyb.LED(3)
BLUE=pyb.LED(4)

PIC= Pin('X7', Pin.OUT_PP)
p_in = Pin('X8', Pin.IN, Pin.PULL_UP)


Ma = Pin('X5', Pin.OUT_PP)
Mb = Pin('X6', Pin.OUT_PP)
Sp = Pin('X9')
tim = Timer(4, freq = 10000)
ch = tim.channel(1, Timer.PWM, pin = Sp)

def motor(speed):
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
    if uart.any():
        YELLOW.on()
        a = chr(uart.readchar())
        if a != '$':
            inn += a
            if len(inn) > 6:
                inn = ""

        else:
            if flag_start:
                if inn == "999999":
                    flag_start = False
                    BLUE.on()
                    PIC.high()
                    delay(500)
                    PIC.low()
            else:
                message = str(1 - p_in.value()) + '$'
                try:
                    if len(inn) == 6 and inn != '999999':
                        rul = int(inn[:3]) - 200
                        speed = int(inn[3:]) - 200
                        print(speed, rul)
                        if rul>65:
                            rul=65
                        if rul<-40:
                            rul=-40
                        motor(speed)
                        servo.angle(rul)



                except ValueError:
                    print("err")

            print(inn)
            inn = ""
            uart.write(message)