#setup section

#Time Library Import
import time

#3D Mouse Libraries/Setup
from spnav import *
spnav_open()

#Cartesian Space to Joint Space Libraries
from math import *
import numpy as np

#Arm Calculation Code Import
import Arm

wmroboarm = Arm.Arm3Link(L = np.array([255,255,1]))

#Arduino Communication
import minimalmodbus
arduino = minimalmodbus.Instrument('/dev/ttyACM0', 1) # port name, slave address (in decimal)
#arduino.debug = True

#Target variables for Arm3Link functions
targety = 0
targetz = 0

#Mouse scaling constant
mouse_scal = 100

#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#Minimum and maximum boundaries in mm

#Delay to let Arduino Connect
time.sleep(2)

arduino.write_register(6, 0, 0)

#=================================================================

#3D Mouse Input loop
try:
    while True:
        
        #Read Potentiometer Values from Arduino
        Shoulder_Pot_Degrees = arduino.read_register(2, 1)
        Elbow_Pot_Degrees = arduino.read_register(3, 1)
        Shoulder_Pot_Raw = arduino.read_register(4, 0)
        Elbow_Pot_Raw = arduino.read_register(5, 0)

        #Convert Degrees to Radians
        Shoulder_Pot_Radians = radians(Shoulder_Pot_Degrees)
        Elbow_Pot_Radians = radians(Elbow_Pot_Degrees)
        
        #Determine if there is change in the mouse
        event = spnav_poll_event()
        
        if (event != None):
            if (event.ev_type == SPNAV_EVENT_MOTION):
                #Aquiring and parsing out translation and rotation variables from the mouse
                tran = event.translation
                rot = event.rotation
                x = tran[0]
                y = tran[1]
                z = tran[2]
                a = rot[0]
                b = rot[1]
                c = rot[2]
                
                #Scale mouse input to correct size
                targety += (y/mouse_scal)
                targetz += (z/mouse_scal)
                #print "{}, {}".format(targety, targetz)

                #print "{}, {}".format(xang, yang)
                
                #Plug Variables into function and store output as angles variable
                wmroboarm.q = [Shoulder_Pot_Radians, Elbow_Pot_Radians, 0]
                angles = wmroboarm.inv_kin(xy = (targety, targetz))
                #print degrees(angles[1])
                #print degrees(angles[2])
                
                #Find angles to set arm at
                Shoulder_Set_Angle = (degrees(angles[0]))
                Elbow_Set_Angle = (degrees(angles[1]))
                
                #Write angles to arduino
                arduino.write_register(0, Shoulder_Set_Angle, 0)
                arduino.write_register(1, Elbow_Set_Angle, 0)
                spnav_remove_events(SPNAV_EVENT_MOTION)
except KeyboardInterrupt:
    print "Whoa there"

#=================================================================

#teardown
arduino.write_register(6, 1, 0)
arduino.close()
spnav_close() 
