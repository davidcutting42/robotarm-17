# WM-Roboarm
This repository is the main repository for David Cutting's Ward Melville Science Olympiad Robot Arm. All code is written by David Cutting unless otherwise noted. 

(C) 2016-2017 David Cutting

## Installation instructions
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

Expand the filesystem and enable the camera

```
sudo reboot
sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose libspnav-dev libspnav0 spacenavd arduino git
sudo pip install spnav
sudo service spacenavd start
cd Desktop
git clone https://github.com/davecutting/WM-Roboarm.git
sudo pip install ikpy
```
