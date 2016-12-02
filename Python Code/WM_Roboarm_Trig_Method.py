import math
import numpy as np
from spnav import *
import time
from Tkinter import *
import minimalmodbus

#Find the serial port for the Arduino Mega
try:
    arduino = minimalmodbus.Instrument('/dev/ttyACM0', 1) # port name, slave address (in decimal)
except:
    try:
        arduino = minimalmodbus.Instrument('/dev/ttyACM1', 1) # port name, slave address (in decimal)
    except:
        arduino = minimalmodbus.Instrument('/dev/ttyACM2', 1) # port name, slave address (in decimal)

#Dimensions of the Robot
H = 240.0 # Base height in mm
F = 255.0 # Femur length in mm
T = 255.0 # Tibia length in mm

# Limit variables (mm)
maxy = 480
miny = 130
minz = 100
maxz = minz + maxy

# Starting Position of the Arm (mm)
y = miny 
z = 250

# Open the 3D mouse channel
spnav_open()

# Wait 4 Seconds for all communications channels to come on board.
time.sleep(4)

#Scaling Variables for mouse to target conversion
mouse_scal = 10.0
mouse_scal2 = 1000.0

# Initialization of the timeout clock to zero
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

                if (y < miny): #Constrain y 
                    y = miny
                if (z < minz): #Constrain z
                    z = minz
                if (y == miny): #Provide a case for y to prevent divide by zero
                    if (z > maxz):
                        z = maxz
                elif ((y-miny)**2 + (z-minz)**2 > (maxy-miny)**2): #Check if outside the circle that we want to constrain it in
                    m = (z - minz) / (y - miny)
                    d = z - m * y
                    r = maxy - miny
                    delta = math.pow(r, 2) * (1 + math.pow(m, 2)) - math.pow((minz - m*miny - d), 2)
                    y = (miny + (minz * m) - (d * m) + math.sqrt(delta)) / (1 + math.pow(m, 2))
                    z = m * y + d
                    print "hello"

                    
                L = math.sqrt(math.pow(y,2) + math.pow(H-z,2)) #Calculate the distance from the base to the end of the arm

                # Two different cases to make sure math is correct for when the end is above and below the base.
                if (z > H):
                    alpha1 = math.acos(y/L) + math.pi/2.0
                else:
                    alpha1 = math.asin(y/L)

                # Calculate angles for potentiometer targets in joint space.
                beta = math.acos((math.pow(L,2) - math.pow(T,2) - math.pow(F,2))/(-2.0*T*F))
                alpha2 = math.acos((math.pow(T,2) - math.pow(F,2) - math.pow(L,2))/(-2.0*F*L))     
                alpha = alpha1 + alpha2

                # Get potentiometer data from the arduino
                shoulderpot = arduino.read_register(2, 1)
                elbowpot = arduino.read_register(3, 1)
                rawshoulder = arduino.read_register(4, 0)
                rawelbow = arduino.read_register(5, 0)

                # Print targets, current positions, etc.
                print "alph: {:.2f}, beta: {:.2f}, y: {:.2f}, z: {:.2f}, shoulder: {}, elbow: {} rawshoulder: {} rawelbow: {} gamma: {}".format(np.degrees(alpha), np.degrees(beta), y, z, shoulderpot, elbowpot, rawshoulder, rawelbow, gamma)

                # Write joint space targets to arduino
                arduino.write_register(0, int(math.degrees(alpha)*10), 0)
                arduino.write_register(1, int(math.degrees(beta)*10), 0)
                arduino.write_register(6, gamma, 0)

                # Remove extra events to avoid lag in spacenav.
                spnav_remove_events(SPNAV_EVENT_MOTION)

                #Reset the timeout variable
                elim = 0;

        #Check to see if timeout has occoured, then set the mouse inputs to zero so drift is eliminated.
        if (event == None):
            elim = elim + 1
        if (elim > 400):
            arduino.write_register(6, 350, 0)
            
except KeyboardInterrupt:
        print "Program Terminated by User"
        
spnav_close() #Close the spacenav channel after program ends
