# Arduino Modbus Register Table (Raspberry pi is master, arduino is slave)
# Register:     Register Name:      Source:         Description:
#   0               xtarget             Master          Target x coordinate (mm)
#   1               ytarget             Master          Target y coordinate (mm)
#   2               bendpreference      Master          Bend preference (0 or 1)
#   3               basetarget          Master          Target angle for base motor (degrees)
#   4               steppersinposition  Slave           0 = last command not executed, 1 = last command completed
#   5               benddirection       Slave           Current bend direction
#   6               mode                Master          0 = idle, 1 = run, 2 = stream (calibrate), 3 = set current position as zero
#   7               servapos            Master          Target position (degrees) of servo A
#   8               servbpos            Master          Target position (degrees) of servo B
#   9               motdangle           Master          Motor d target position (degrees)

from Tkinter import *
import time
import minimalmodbus
f = open("waypoints.txt", "r")

#Find the serial port for the Arduino Mega
try:
    arduino = minimalmodbus.Instrument('/dev/ttyACM0', 1) # port name, slave address (in decimal)
except:
    try:
        arduino = minimalmodbus.Instrument('/dev/ttyACM1', 1) # port name, slave address (in decimal)
    except:
        try:
            arduino = minimalmodbus.Instrument('/dev/ttyACM2', 1) # port name, slave address (in decimal)
        except:
            arduino = minimalmodbus.Instrument('/dev/ttyACM3', 1) # port name, slave address (in decimal)

arduino.serial.baudrate = 57600

#Create Tkinter Window
root = Tk()



xtarget = 0
ytarget = 249 * 2
bendpreference = 1
basetarget = 0
servoapos = 120 #opposite is 45 (Fork)
servobpos = 180 #opposite is 20 (Flip)
motdangle = 0
mode = 2

def updatearduinoregisters(event): #Function for stepper control and spacenav input
    registers = [xtarget+1000, ytarget+1000, bendpreference, basetarget, 0, 0, mode, servoapos, servobpos, motdangle]
    #print registers
    arduino.write_registers(0, registers)
    root.update_idletasks()

xval = Label(root, text=xtarget)
xval.grid(row=0, column=2)

def xdecrement(event):
    global xtarget
    xtarget -= 1
    xval.config(text=xtarget)
    
def xincrement(event):
    global xtarget
    xtarget += 1
    xval.config(text=xtarget)

xlab = Label(root, text="X: ")
xlab.grid(row=0, column=0)

root.bind("<Left>",xdecrement)
root.bind("<Right>",xincrement)


yval = Label(root, text=ytarget)
yval.grid(row=1, column=2)

def ydecrement(event):
    global ytarget
    ytarget -= 1
    yval.config(text=ytarget)
    
def yincrement(event):
    global ytarget
    ytarget += 1
    yval.config(text=ytarget)
    
    
ylab = Label(root, text="Y: ")
ylab.grid(row=1, column=0)

root.bind("<Up>",yincrement)
root.bind("<Down>",ydecrement)


bendval = Label(root, text=bendpreference)
bendval.grid(row=2, column=2)
 
def benddecrement(event):
    global bendpreference
    bendpreference = 0
    bendval.config(text=bendpreference)
    
def bendincrement(event):
    global bendpreference
    bendpreference = 1
    bendval.config(text=bendpreference)
    
bendlab = Label(root, text="Bend: ")
bendlab.grid(row=2, column=0)

root.bind("<F2>",bendincrement)
root.bind("<F1>",benddecrement)

baseval = Label(root, text=basetarget)
baseval.grid(row=3, column=2)
    
def basedecrement(event):
    global basetarget
    basetarget -= 1
    if(basetarget < 0):
        basetarget = 0
    baseval.config(text=basetarget)
    
    
def baseincrement(event):
    global basetarget
    basetarget += 1
    if(basetarget > 95):
        basetarget = 95
    baseval.config(text=basetarget)
    
baselab = Label(root, text="Base: ")
baselab.grid(row=3, column=0)

root.bind("<Shift-Up>",baseincrement)
root.bind("<Shift-Down>",basedecrement)

servoaval = Label(root, text=servoapos)
servoaval.grid(row=4, column=2)
  
def servoadecrement(event):
    global servoapos
    servoapos -= 1
    if(servoapos < 45):
        servoapos = 45
    servoaval.config(text=servoapos)
      
def servoaincrement(event):
    global servoapos
    servoapos += 1
    if(servoapos > 120):
        servoapos = 120
    servoaval.config(text=servoapos)
    
servoalab = Label(root, text="Servo A: ")
servoalab.grid(row=4, column=0)

root.bind("9",servoadecrement)
root.bind("0",servoaincrement)

servobval = Label(root, text=servobpos)
servobval.grid(row=5, column=2)

def servobdecrement(event):
    global servobpos
    servobpos -= 1
    if(servobpos < 20):
        servobpos = 20
    servobval.config(text=servobpos)
    
    
def servobincrement(event):
    global servobpos
    servobpos += 1
    if(servobpos > 180):
        servobpos = 180
    servobval.config(text=servobpos)
    

servoblab = Label(root, text="Servo B: ")
servoblab.grid(row=5, column=0)

root.bind("[",servobdecrement)
root.bind("]",servobincrement)

motdval = Label(root, text=motdangle)
motdval.grid(row=6, column=2)

def motddecrement(event):
    global motdangle 
    motdangle -= 1
    if(motdangle < 0):
        motdangle = 0
    motdval.config(text=motdangle)
    
def motdincrement(event):
    global motdangle 
    motdangle += 1
    if(motdangle > 360):
        motdangle = 360
    motdval.config(text=motdangle)  

motdlab = Label(root, text="Motor D: ")
motdlab.grid(row=6, column=0)

root.bind(",",motddecrement)
root.bind(".",motdincrement)


def printresults(event):
    global xtarget
    global ytarget
    global bendpreference
    global basetarget
    global servoapos
    global servobpos
    global motdangle
    print("{}, {}, {}, {}, {}, {}, {}, {}, {}, {}".format(xtarget, ytarget, bendpreference, basetarget, 0, 0, mode, servoapos, servobpos, motdangle))


root.bind("p",printresults)

def zero():
    global xtarget
    global ytarget
    global bendpreference
    global basetarget
    global servoapos
    global servobpos
    global motdangle
    global mode
    xtarget = 0
    ytarget = 249 * 2
    bendpreference = 1
    basetarget = 0
    servoapos = 120 #opposite is 45 (Fork)
    servobpos = 180 #opposite is 20 (Flip)
    motdangle = 0
    mode = 3
    updatearduinoregisters(0)
    mode = 2

zero = Button(root, text="Zero", command=zero, height=2, width=8, bg='navy blue', fg='white')
zero.grid(row=7, column=2)

root.bind("<Enter>",updatearduinoregisters)

def runnext():
    raw = f.readline()
    wapoint = [int(x) for x in raw.split(',') if x]
    wapoint[0] += 1000
    wapoint[1] += 1000
    print wapoint
    arduino.write_registers(0, wapoint)
    root.update_idletasks()

runnext = Button(root, text="Next", command=runnext, height=2, width=8, bg='green')
runnext.grid(row=7, column=0)

#root.after(1, updateSteppers)
root.mainloop()
