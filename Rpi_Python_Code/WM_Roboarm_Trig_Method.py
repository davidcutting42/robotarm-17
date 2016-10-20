import math
import numpy as np
from spnav import *
import time

import minimalmodbus

try:
    arduino = minimalmodbus.Instrument('/dev/ttyACM1', 1) # port name, slave address (in decimal)
except:
    arduino = minimalmodbus.Instrument('/dev/ttyACM2', 1) # port name, slave address (in decimal)

y = 180.0
z = -70.0
gamma = 3.14
zoffset = 240.0
femur = 255.0
tibia = 255.0

# Open the 3D mouse instrunment
spnav_open()

time.sleep(2)

mouse_scal = 10.0
mouse_scal2 = 1000.0

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
                my = tran[1]
                mz = tran[2]
                ma = rot[0]
                mb = rot[1]
                mc = rot[2]

                #Scale mouse input to correct size
                z += (my/mouse_scal)
                y += (mz/mouse_scal)
                gamma += (-mb/mouse_scal2)

                if (y < 50):
                    y = 50.0
                if (z > zoffset):
                    z = zoffset
                if (y > 400):
                    y = 400.0
                if (z < -400):
                    z = -400.0
                if (gamma > math.pi/2):
                    gamma = math.pi/2
                if (gamma < -math.pi/2):
                    gamma = -(math.pi/2)
                
                L = math.sqrt(math.pow(y,2) + math.pow(z,2))
                
                if (z > 0):
                    alpha1 = math.atan(y/z)
                elif (z < 0):
                    alpha1 = math.pi/2+math.atan(z/y)
                else:
                    alpha1 = math.pi/2.0

                    
                beta = math.acos((math.pow(L,2) - math.pow(tibia,2) - math.pow(femur,2))/(-2*tibia*femur))
                alpha2 = math.acos((math.pow(tibia,2) - math.pow(femur,2) - math.pow(L,2))/(-2*femur*L))     
                alpha = alpha1 + alpha2

                shoulderpot = arduino.read_register(2, 1)
                elbowpot = arduino.read_register(3, 1)
                rawshoulder = arduino.read_register(4, 0)
                rawelbow = arduino.read_register(5, 0)
                
                print "alph: {:.2f}, beta: {:.2f}, y: {:.2f}, z: {:.2f}, shoulder: {}, elbow: {} rawshoulder: {} rawelbow: {}".format(np.degrees(alpha), np.degrees(beta), y, z, shoulderpot, elbowpot, rawshoulder, rawelbow)

                #try:
                arduino.write_register(0, int(math.degrees(alpha)*10), 0)
                arduino.write_register(1, int(math.degrees(beta)*10), 0)
                #arduino.write_register(2, math.degrees(gamma), 1)
                #arduino.write_register(2, int(math.degrees(0)*10), 0)
                #except ValueError:
                #    print "Go back to Algebra 1"
                #except:
                #    print "error"
                
                spnav_remove_events(SPNAV_EVENT_MOTION)          
except KeyboardInterrupt:
        print "Whoa there"
        
spnav_close() 

