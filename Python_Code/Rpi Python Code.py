# Arduino Modbus Register Table
# Register:     Register Name:      Source:         Description:
#   0               alpha               Master          Target angle for shoulder
#   1               beta                Master          Target angle for elbow
#   2               shoulderpot         Slave           Current angle of shoulder (debug only)
#   3               elbowpot            Slave           Current angle of elbow (debug only)
#   4               rawshoulder         Slave           Current analogRead value of Shoulder (debug only)
#   5               rawelbow            Slave           Current analogRead value of Elbow (debug only)
#   6               gamma               Master          Rotation of 3D mouse about z axis
#   7               servoapos           Master          Target angle of Servo A
#   8               servobpos           Master          Target angle of Servo B
#   9               sidee              Master          Penny side (0=empty, 1=heads, 2=tails)Target angle of Servo C
#   10              --                  Master          Reserved for additional servo control   
        

# Laptop Modbus Register Table
# Register:     Register Name:      Source:         Description:
#   0               side                Slave           Penny side (0=empty, 1=heads, 2=tails)

from Tkinter import *
import math
import numpy as np
from spnav import *
import time
import minimalmodbus

#laptop = minimalmodbus.Instrument('192.168.5.2', 1, mode='tcp') # port name, slave address (in decimal)

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

#Create Tkinter Window
root = Tk()

#Create servo position variables
servo1pos=90
servo2pos=90
servo3pos=90

sidee = 0

#Functions for modifying servo A angle
def servoAleft():
    global servo1pos
    servo1pos -= 90
    updateServoA()
def servoAright():
    global servo1pos
    servo1pos += 90
    updateServoA()
    
#Functions for modifying servo B angle    
def servoBleft():
    global servo2pos
    servo2pos -= 90
    updateServoB()
def servoBright():
    global servo2pos
    servo2pos += 90
    updateServoB()
    
#Functions for modifying servo C angle    
def servoCleft():
    global sidee
    sidee = 1
    updateServoD()
def servoCright():
    global sidee
    sidee = 2
    updateServoD()

#Servo A button/label initialization and configuration
servoalabel = Label(root, text="Scoop")
servoaleft = Button(root, text="<<", command=servoAleft, height=3, width=8)
servoaangle = Label(root, text=servo1pos)
servoaright = Button(root, text=">>", command=servoAright, height=3, width=8)
#Servo A control object location definitions using grids.
servoalabel.grid(row=0, column=0)
servoaleft.grid(row=0, column=1)
servoaangle.grid(row=0, column=2)
servoaright.grid(row=0, column=3)

#Servo B button/label initialization and configuration
servoblabel = Label(root, text="Lift")
servobleft = Button(root, text="<<", command=servoBleft, height=3, width=8)
servobangle = Label(root, text=servo2pos)
servobright = Button(root, text=">>", command=servoBright, height=3, width=8)
#Servo B control object location definitions using grids.
servoblabel.grid(row=1, column=0)
servobleft.grid(row=1, column=1)
servobangle.grid(row=1, column=2)
servobright.grid(row=1, column=3)

#Servo C button/label initialization and configuration
servoclabel = Label(root, text="Flip")
servocleft = Button(root, text="HEADS", command=servoCleft, height=3, width=8)
servocangle = Label(root, text=sidee)
servocright = Button(root, text="TAILS", command=servoCright, height=3, width=8)
#Servo C control object location definitions using grids.
servoclabel.grid(row=3, column=0)
servocleft.grid(row=3, column=1)
servocangle.grid(row=3, column=2)
servocright.grid(row=3, column=3)

def updateServoD():
    global sidee

    servocangle.config(text=sidee)

    arduino.write_register(9, sidee, 0)

def updateServoA():
    global servo1pos
    global servoaangle
    
    if (servo1pos > 180):
        servo1pos = 180
    if (servo1pos < 0):
        servo1pos = 0

    servoaangle.config(text=servo1pos)
    arduino.write_register(7, servo1pos, 0)

def updateServoB():
    global servo2pos
    global servobangle
    
    if (servo2pos > 180):
        servo2pos = 180
    if (servo2pos < 0):
        servo2pos = 0

    servobangle.config(text=servo2pos)
    arduino.write_register(8, servo2pos, 0)

def updateSteppers(): #Function for stepper control and spacenav input

    global H
    global F
    global T
    global maxy
    global miny
    global minz
    global maxz
    global y
    global z
    global mouse_scal
    global mouse_scal2
    global elim
    
    try:
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

                #ide = laptop.read_register(0, 0)

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
    
    root.after(1, updateSteppers)

root.after(1, updateSteppers)
root.mainloop()
