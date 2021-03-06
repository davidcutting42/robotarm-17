﻿# robotarm-17
David Cutting's Ward Melville Science Olympiad robot arms.

(C) 2016-2017 David Cutting

## Installation Instructions
###On Windows Desktop:
Download Raspbian Image from Rpi Website

Unzip onto Desktop

Plug in an minimum 16GB SD card

Download and run rufus, then select the DD image flashing option with the unzipped image file.

###On Raspberry Pi:

Boot with SD card, then run

```
sudo raspi-config
```

Expand the filesystem, then reboot and run the following code:

```
sudo apt-get install python-numpy arduino git
cd Desktop
git clone https://github.com/davecutting/WM-Roboarm.git
sudo pip install minimalmodbus
```
Download the Modbus-Master-Slave library and unzip it to pi/sketchbook/libraries
Required arduino libraries: Wire, Adafruit_PWMServoDriver, ModbusRtu (https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino), SoftwareSerial, ams_as5048b

Enable the VNC option in the preferences tab on the arduino, and make sure to set your arduino to automatically connect to your wifi hotspot on your phone at the tournament. That's it! 
