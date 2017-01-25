# WM-Roboarm
This repository is the main repository for David Cutting's Ward Melville Science Olympiad Robot Arm. All code is written by David Cutting unless otherwise noted. 

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

Expand the filesystem and enable the camera, then reboot and run the following code:

```
sudo apt-get install python-numpy libspnav-dev libspnav0 spacenavd arduino git
cd Desktop
git clone https://github.com/davecutting/WM-Roboarm.git
sudo pip install minimalmodbus spnav
sudo service spacenavd start
```

You will need to install the Adafruit TFT kernel over the raspbian installation. I would not reccomend using their image file because if is not the latest version of raspbian most of the time and does not have the niceties associated with PIXEL, etc. 

Instead I would suggest that you use the DIY installer script that they provide.

The instructions for installing the kernel can be found here: https://learn.adafruit.com/adafruit-pitft-28-inch-resistive-touchscreen-display-raspberry-pi/easy-install

Install the remote camera viewer setup as per http://elinux.org/RPi-Cam-Web-Interface#Loading
You should remove the subfolder and not put a username and password in when prompted during setup. Then make the IP address on the Rpi and Laptop you are using to look at the video static and in the same subaddress (I used 192.168.5.1 for the pi and 192.168.5.2 for the laptop, with a subnet mask of 255.255.255.0)

Download the Modbus-Master-Slave library and unzip it to pi/sketchbook/libraries

## Coding/Debug Mode
After the robot has completed its boot process, press Ctrl+Alt+F1 to exit the GUI in the piTFT screen. The GUI will freeze on the piTFT and the terminal will show up on the HDMI screen. Start the GUI on the HDMI monitor:

```
startx -- -layout HDMI
```

You should be presented with a desktop similar (albeit not the same size as) the one on the piTFT. Congrats!


Tutorial on getting it set up: 
https://www.raspberrypi.org/forums/viewtopic.php?f=44&t=91764&start=25

Possible Problems
If you get any kind of error related to authority to lock the .Xauthority file, run this command to give ownership of all pi files to the pi user.

```	
sudo chown pi:pi /home/pi
```

Make sure you do not insert “sudo” in front of the startx command. Doing so will result in you ending up in the root desktop, not the pi desktop. And it will screw up some other stuff too.

## Troubleshooting
For the pi I used the IP address 192.168.5.1 and on the laptop I used 192.168.5.2. I used a subnet mask of 255.255.255.0.

To calibrate the potentiometers, you need to have a way of knowing where the arm is. I took pictures and noted the readout as explained in my blog post here: http://davidjcutting.com/2016/11/29/potentiometer-woes/ but a protractor custom made to mount on the arm would have been a better solution had I had more time. This will probably be added later, or the potentiometers changed out for an absolute position digital encoder.

Required arduino libraries: Wire, Adafruit_PWMServoDriver, ModbusRtu (https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino), SoftwareSerial, ams_as5048b 