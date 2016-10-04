#setup section

#3D Mouse Libraries/Setup
from spnav import *
spnav_open()
import Arm

#Cartesian Space to Joint Space Libraries
import math
import numpy as np

#Arduino Serial Communication
import serial
ser = serial.Serial('/dev/ttyACM0', 115200)

x = 0
y = 0
z = 0
a = 0
b = 0
c = 0
angles = [math.pi/4, math.pi/4, 0]

#=================================================================

#3D Mouse Input loop
try:
    while True:
        event = spnav_poll_event()
        if (event != None):
            if (event.ev_type == SPNAV_EVENT_MOTION):
                tran = event.translation
                rot = event.rotation
                x = tran[0]
                y = tran[1]
                z = tran[2]
                a = rot[0]
                b = rot[1]
                c = rot[2]
                yrad = y*180/np.pi
                zrad = z*180/np.pi
                angles = Arm.Arm3Link(q = [angles[0], angles[1], 0],L = np.array([255,255,1])).inv_kin(xy = (y, z))
                shoulder = ((int(angles[0]*180/math.pi) * 10**3) / 10.0**3)
                elbow = ((int(angles[1]*180/math.pi) * 10**3) / 10.0**3)
            transmission = [shoulder, elbow]
            transstring = str(transmission).strip('[]')
            print transstring
            ser.write(transstring)
except KeyboardInterrupt:
    print "Whoa there"




#=================================================================

#teardown
spnav_close() 
