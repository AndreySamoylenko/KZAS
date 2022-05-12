# Program Explanations

![main.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/main.py) is a helper program executable by pyboard that takes messages from the raspberry and converts them into a servo angle and motor speed.
![qualification](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/qualification.py), ![final1.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/final1.py), ![final2.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/final2.py) these programs are uploaded to raspberry and have a similar structure.
All of them can be divided into 3 stages of work:

1. button waiting
2. travel
3. finish

(in the program, the stages are defined by the "state" variable)

In the first step, the raspberry sends the message "$999999" which the pyboard reads and waits for the value to change while sending the button readings in parallel.
When the button is pressed the pyboard sends "1$".
raspberry accepts this and moves on to the next stage.

The stage of travel for all three programs is different.

In ![qualification](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/qualification.py), the second stage is the passage with the help of a proportional-differential regulator with two sensors and with parallel reading of the so-called intersections (orange and blue lines).
The PD controller first calculates the deviation (e=sr1-sr2) and then, based on it, calculates the control action (u = (e * kp + (e - e_old) * kd) (e_old is the previous deviation).
the control action is sent with the first three digits of the message.

In the ![final1.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/final1.py) there is everything the same as in the ![qualification](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/qualification.py), the functions for bypassing the cubes have been added.
In the driving stage, a new huge sensor appears on the screen (although there are actually two) that reads the cubes and returns their color and position on the camera.
And if the robot sees the cube, then it, ignoring the black lines, calculates a new deviation (e = (240 + hg * 1.3) -srg (hg is the position of the cube in y and srg is the position in x)).

![final2.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/final2.py) is an improved version of ![final1.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/final1.py).
Basically, useful features have been added to it that allow you to improve the result in the form of acceleration if there are no cubes and an improved perspective.




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
