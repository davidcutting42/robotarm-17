# WM-Roboarm
This repository is the main repository for David Cutting's Ward Melville Science Olympiad Robot Arm. All code is written by David Cutting unless otherwise noted. 

(C) 2016-2017 David Cutting

## Coding/Debug Mode
After the robot has completed its boot process, press Ctrl+Alt+F1 to exit the GUI in the piTFT screen. The GUI will freeze on the piTFT and the terminal will show up on the HDMI screen. Start the GUI on the HDMI monitor:


startx -- -layout HDMI


You should be presented with a desktop similar (albeit not the same size as) the one on the piTFT. Congrats!


Tutorial on getting it set up: 
https://www.raspberrypi.org/forums/viewtopic.php?f=44&t=91764&start=25

Possible Problems
If you get any kind of error related to authority to lock the .Xauthority file, run this command to give ownership of all pi files to the pi user.

```	
sudo chown pi:pi /home/pi
```

Make sure you do not insert “sudo” in front of the startx command. Doing so will result in you ending up in the root desktop, not the pi desktop.


## Installation instructions (OUTDATED AND NEED TO BE UPDATED ONCE STABLE)
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
sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose libspnav-dev libspnav0 spacenavd arduino git
cd Desktop
git clone https://github.com/davecutting/WM-Roboarm.git
sudo pip install ikpy minimalmodbus spnav
sudo service spacenavd start
```

Download the Modbus-Master-Slave library and unzip it to pi/sketchbook/libraries

