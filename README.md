# Сonnecting to pyboard

To connect to the pyboard, you need to connect the micro usb cable to your computer, and open the pyboard as a USB flash drive in the explorer.
In the Pyboard tab there will be two files ![main.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/main.py) and boot.py. the ![main.py](https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/main.py) file is our program that pyboard will execute.

# Connecting to raspberry pi

connecting to the raspberry is a little more difficult but if you have connected the pyboard it will be easier.
first you need to turn on the power on the robot and wait until the pyboard emits "beeeeeeeep".
this sound means that the raspberry is ready to work and distributes the Internet.
after that, we search for our raspberry's network and connect to it.
once you are connected to the network, you need to run the ![start_robot.py]https://github.com/AndreySamoylenko/WRO_NanoGayka/blob/main/start_robot.py program on your computer.
choose your robot.
click "load start" and select the appropriate one in the window with files.
voila, your program is loaded and if the blue and yellow LEDs on the pyboard are lit, then the pyboard is successfully receiving messages from the raspberry.
