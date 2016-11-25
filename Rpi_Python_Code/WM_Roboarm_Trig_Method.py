import math
import numpy as np
from spnav import *
import time

import minimalmodbus

try:
    arduino = minimalmodbus.Instrument('/dev/ttyACM0', 1) # port name, slave address (in decimal)
except:
    try:
        arduino = minimalmodbus.Instrument('/dev/ttyACM1', 1) # port name, slave address (in decimal)
    except:
        arduino = minimalmodbus.Instrument('/dev/ttyACM2', 1) # port name, slave address (in decimal)

gamma = 3.14
H = 240.0 # Base height in mm
F = 255.0 # Femur length in mm
T = 255.0 # Tibia length in mm
y = 100
z = H

# Open the 3D mouse instrunment
spnav_open()

time.sleep(4)

mouse_scal = 10.0
mouse_scal2 = 1000.0

elim = 0

try:
    while True:
        #Determine if there is change in the mouse
        event = spnav_poll_event()

        if (event != None):
            if (event.ev_type == SPNAV_EVENT_MOTION):
                #Aquiring and parsing out translation and rotation variables from the mouse
                tran = event.translation
                rot = event.rotation
                mx = tran[0]
                mz = tran[1]
                my = tran[2]
                ma = rot[0]
                mb = rot[1]
                mc = rot[2]

                #Scale mouse input to correct size
                z += (mz/mouse_scal)
                y += (my/mouse_scal)
                gamma = (mb+350)

                if (y < 50):
                    y = 50.0
                if (z > H+F):
                    z = H+F
                if (y > T+F):
                    y = T+F
                if (z < 0):
                    z = 0
                
                L = math.sqrt(math.pow(y,2) + math.pow(H-z,2))
                
                if (z > H):
                    alpha1 = math.acos(y/L) + math.pi/2.0
                else:
                    alpha1 = math.asin(y/L)

                    
                beta = math.acos((math.pow(L,2) - math.pow(T,2) - math.pow(F,2))/(-2.0*T*F))
                alpha2 = math.acos((math.pow(T,2) - math.pow(F,2) - math.pow(L,2))/(-2.0*F*L))     
                alpha = alpha1 + alpha2

                shoulderpot = arduino.read_register(2, 1)
                elbowpot = arduino.read_register(3, 1)
                rawshoulder = arduino.read_register(4, 0)
                rawelbow = arduino.read_register(5, 0)
                
                print "alph: {:.2f}, beta: {:.2f}, y: {:.2f}, z: {:.2f}, shoulder: {}, elbow: {} rawshoulder: {} rawelbow: {} gamma: {}".format(np.degrees(alpha), np.degrees(beta), y, z, shoulderpot, elbowpot, rawshoulder, rawelbow, gamma)

                arduino.write_register(0, int(math.degrees(alpha)*10), 0)
                arduino.write_register(1, int(math.degrees(beta)*10), 0)
                arduino.write_register(6, gamma, 0)

                spnav_remove_events(SPNAV_EVENT_MOTION)

                elim = 0;
        if (event == None):
            elim = elim + 1
        if (elim > 400):
            arduino.write_register(6, 350, 0)
except KeyboardInterrupt:
        print "Whoa there"
        
spnav_close() 

