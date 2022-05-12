# Program Explanations

![main.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/main.py) is a helper program executable by pyboard that takes messages from the raspberry and converts them into a servo angle and motor speed.
![qualification](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/qualification.py), ![final1.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/final1.py), ![final2.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/final2.py) programs are uploaded to raspberry and have similar structure.
All of them can be divided into 3 stages of work:

1. button waiting
2. travel
3. finish

(in the program, the stages are defined by the "state" variable)

In the first step, the raspberry sends the message "999999$" which the pyboard reads and waits for the value to change while sending the button readings in parallel.
When the button is pressed the pyboard sends "1$".
raspberry accepts this and moves on to the next stage.

The stage of travel for all three programs is different.

In ![qualification](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/qualification.py), the second stage is the passage with the help of a proportional-differential regulator with two sensors and with parallel reading of the so-called intersections (orange and blue lines).
The PD controller first calculates the deviation (e=sr1-sr2) and then, based on it, calculates the control action (u = (e * kp + (e - e_old) * kd) (e_old is the previous deviation).
the control action is sent with the first three digits of the message.

![final1.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/final1.py) works in a similiar way as ![qualification](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/qualification.py) but perfoms a different set of actions , the functions for bypassing the cubes have been added.
In the driving stage, a new huge sensor appears on the screen (although there are actually two) that reads the cubes and returns their color and position on the camera.
And if the robot sees the cube, then it calculates a new deviation (e = (240 + hg * 1.3) -srg (hg is the position of the cube in y and srg is the position in x)), ignoring the black lines.

![final2.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/final2.py) is an improved version of ![final1.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/final1.py).
Basically,an improved perspective has been added, as wel as useful features to it that allow you to improve the result by acceleration if there are no cubes .

All the previous programs were for the passage of the route, the subsequent programs will be useful to us in a different way.

![RobotAPI.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/RobotAPI.py) creates a class that is responsible for reading the camera and, in principle, the operation of the raspberry and the ability to connect to it.

![start_robot.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/start_robot.py) using ![RobotAPI.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/RobotAPI.py) makes it easier for us to use the robot.]
The computer is connected to the raspberry via IP using Wi-Fi network.
In the window created by the program, we observe 6 buttons: "load start", "start", "stop", "raw" , "video" and "connect to robot".
If you run ![start_robot.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/start_robot.py) and try to do something, the program will print a "select robot" message in red.
This means that before you upload something, you need to understand where to upload it.
In this case, you click on the "connect to robot" button and select your robot (you must first connect to the Wi-Fi of our robot).
After that, we upload the necessary program on the raspberry ("load start" button) or, if it is loaded, restart it ("start" button).
With the "video" button you can display the video from the robot's camera on the screen of your laptop in a separate window.
The same window will display telemetry.
The "raw" button launches a test file that displays just video from the camera without telemetry.
The "stop" button stops the running program and camera output.


# Connecting to pyboard

To connect to the pyboard, you need to connect the micro usb cable to your computer, and open the pyboard as a USB flash drive in the explorer.
In the Pyboard tab there will be two files ![main.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/main.py) and boot.py. the ![main.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/main.py) file is our program that pyboard will execute.
Copy this file to pycharm, edit it if required and copy it back to pyboard.

# Connecting to raspberry pi

Connecting to the raspberry is a little more complicated but if you have connected the pyboard it will be easier.
First you need to turn on the power on the robot and wait until the pyboard emits "beeeeeeeep".
This sound means that the raspberry is ready to work and distributes the Internet.
After that, we search for our raspberry's network and connect to it.
once you are connected to the network, you need to run the ![start_robot.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/start_robot.py) program on your computer.
Choose your robot.
Click "load start" and select the appropriate one in the window with files.
Voila, your program is loaded and if the blue and yellow LEDs on the pyboard are lit, then the pyboard is successfully receiving messages from the raspberry.

# Running the program on the Raspberry Pi.

Once you have selected a program in ![start_robot.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/start_robot.py) the program can be started by pressing a button on the robot.
But if you turned the robot off and on, then your program will not start.
In this case, the program that is written in ![autostart.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/autostart.py) is launched.
Therefore, you can write the desired program in ![autostart.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/autostart.py) and you will not need to restart the program through ![start_robot.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/start_robot.py).

# If something goes wrong

in case you have encountered difficulties or the robot does not work, then it is recommended to perform the following actions.
In a very likely situation that the computer's and raspberry's IP did not match and the robot does not connect, first connect to the Wi-Fi network of your raspberry.
After that, open the properties of the network and find an IP there.
this ip must be thrown into ![RobotAPI.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/RobotAPI.py).

If the robot does not work correctly, check the voltage on the voltmeter. If it is below 7.5V, immediately change the batteries (it is recommended to change the batteries as soon as the voltage drops to 10.5V).

If the raspberry and the pyboard do not work simultaneously, then there is no contact on the stabilizer.
