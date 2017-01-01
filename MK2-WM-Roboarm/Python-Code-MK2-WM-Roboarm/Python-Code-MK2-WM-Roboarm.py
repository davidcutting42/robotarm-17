# Arduino Modbus Register Table
# Register:     Register Name:      Source:         Description:
#   0               xtarget             Master          Target angle for shoulder
#   1               ytarget             Master          Target angle for elbow
#   2               bendpreference      Master           Current angle of shoulder
#   3               basetarget          Master           Current angle of elbow
#   4               xposition           Slave           Current analogRead Shoulder
#   5               yposition           Slave           Current analogRead Elbow
#   6               benddirection       Slave          Rotation of 3D mouse 
#   7               baseangle           Slave          Target angle of Servo A

from Tkinter import *
import math
import numpy as np
import time
import minimalmodbus

#Find the serial port for the Arduino Mega
try:
    arduino = minimalmodbus.Instrument('/dev/ttyACM0', 1) # port name, slave address (in decimal)
except:
    try:
        arduino = minimalmodbus.Instrument('/dev/ttyACM1', 1) # port name, slave address (in decimal)
    except:
        arduino = minimalmodbus.Instrument('/dev/ttyACM2', 1) # port name, slave address (in decimal)

arduino.serial.baudrate = 57600

# Wait 2 Seconds for all communications channels to come on board.
time.sleep(2)

#Create Tkinter Window
root = Tk()

h = 249.2
u = 249.2

homepos = (0, h + u, 0, 90) 
stack1 = (-280, -213, 0, 0)
stack2 = (85, 68, 0, 0)
stack3 = (295, 277, 1, 0)
stack4 = (85, 487, 1, 0)
stack5 = (-124, 277, 1, 0)
targetcenter = (85, 277, 1, 0)
    

def gotoposition(pos):
    global position
    position = pos
    updatearduinoregisters()

def updatearduinoregisters():
    global position
    arduino.write_register(0, position[0], 0)
    arduino.write_register(1, position[1], 0)
    arduino.write_register(2, position[2], 0)
    arduino.write_register(3, position[3], 0)

#def readarduinoregisters(): 


position1button = Button(root, text="Home", command=gotoposition(homepos), height=3, width=8, bg='orange red')
position1button.grid(row=0, column=1)

position2button = Button(root, text="Stack 1", command=gotoposition(stack1), height=3, width=8, bg='orange red')
position2button.grid(row=0, column=2)

position3button = Button(root, text="Stack 2", command=position3gotoposition(stack2), height=3, width=8, bg='orange red')
position3button.grid(row=0, column=3)

position4button = Button(root, text="Stack 3", command=position4gotoposition(stack3), height=3, width=8, bg='orange red')
position4button.grid(row=0, column=4)

position5button = Button(root, text="Stack 4", command=position5gotoposition(stack4), height=3, width=8, bg='orange red')
position5button.grid(row=0, column=5)

position6button = Button(root, text="Stack 5", command=position6gotoposition(stack5), height=3, width=8, bg='orange red')
position6button.grid(row=0, column=6)

position7button = Button(root, text="Target Center", command=position7gotoposition(targetcenter), height=3, width=8, bg='orange red')
position7button.grid(row=0, column=1)

def updateServoA():
    global servo1pos
    
    if (servo1pos > 180):
        servo1pos = 180
    if (servo1pos < 0):
        servo1pos = 0

    arduino.write_register(7, servo1pos, 0)

def updateSteppers(): #Function for stepper control and spacenav input

    # Get potentiometer data from the arduino
    shoulderpot = arduino.read_register(2, 1)
    elbowpot = arduino.read_register(3, 1)
    rawshoulder = arduino.read_register(4, 0)
    rawelbow = arduino.read_register(5, 0)

    # Write joint space targets to arduino
    arduino.write_register(0, int(math.degrees(alpha)*10), 0)
    arduino.write_register(1, int(math.degrees(beta)*10), 0)
    arduino.write_register(6, gamma, 0)
    
    root.after(1, updateSteppers)

root.after(1, updateSteppers)
root.mainloop()
