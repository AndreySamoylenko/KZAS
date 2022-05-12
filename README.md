# Program Explanations

main.py is a helper program executable by pyboard that takes messages from the raspberry and converts them into a servo angle and motor speed.
qualification.py final1.py final2.py these programs are uploaded to raspberry and have a similar structure.
All of them can be divided into 3 stages of work:

1. button waiting
2. travel
3. finish
(in the program, the stages are defined by the "state" variable)

In the first step, the raspberry sends the message "$999999" which the pyboard reads and waits for the value to change while sending the button readings in parallel.
When the button is pressed the pyboard sends "1$".
raspberry accepts this and moves on to the next stage.

The stage of travel for all three programs is different.





# Сonnecting to pyboard

To connect to the pyboard, you need to connect the micro usb cable to your computer, and open the pyboard as a USB flash drive in the explorer.
In the Pyboard tab there will be two files ![main.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/main.py) and boot.py. the ![main.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/main.py) file is our program that pyboard will execute.
Copy this file to pycharm, change it if required and copy it back to pyboard.

# Connecting to raspberry pi

Connecting to the raspberry is a little more difficult but if you have connected the pyboard it will be easier.
First you need to turn on the power on the robot and wait until the pyboard emits "beeeeeeeep".
This sound means that the raspberry is ready to work and distributes the Internet.
After that, we search for our raspberry's network and connect to it.
once you are connected to the network, you need to run the ![start_robot.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/start_robot.py) program on your computer.
Choose your robot.
Click "load start" and select the appropriate one in the window with files.
Voila, your program is loaded and if the blue and yellow LEDs on the pyboard are lit, then the pyboard is successfully receiving messages from the raspberry.

# Running the program on the Raspberry Pi.

Once you have selected a program in ![start_robot.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/start_robot.py) the program can be started by pressing a button on the robot.
But if you turned off and on the robot, then your program will not start.
In this case, the program that is written in ![autostart.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/autostart.py) is launched.
Therefore, you can write the desired program in ![autostart.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/autostart.py) and you will not need to restart the program through ![start_robot.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/start_robot.py).
