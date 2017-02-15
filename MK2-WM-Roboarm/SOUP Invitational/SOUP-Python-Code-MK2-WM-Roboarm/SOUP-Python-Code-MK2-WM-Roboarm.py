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
#   10              xymode              Master          Mode of X any X axes... 1=Fast, 0=Slow
#   11              liftmode            Master          Mode of Lift motor... 1=Fast, 0=Slow
#   12              encoderadeg         Slave           Current angle outputted by encoder A
#   13              encoderbdeg         Slave           Current angle outputted by encoder B
#   14              encoderddeg         Slave           Current angle outputted by encoder D
#   15              servcpos            Master          Target position (degrees) of servo C

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
servoapos = 130 #opposite is 45 (Fork)
servobpos = 145 #opposite is 20 (Flip)
motdangle = 0
mode = 2
speed=0
rawlast = 0
xymode = 0
liftmode = 0
stopping = 0
encoderadeg = 0
encoderbdeg = 0
encoderddeg = 0
servocpos = 90

def updatearduinoregisters(): #Function for stepper control and spacenav input
    global xtarget, ytarget, bendpreference, basetarget, servoapos, servobpos, motdangle, xymode, liftmode, mode, servocpos

    registers = [xtarget+1000, ytarget+1000, bendpreference, basetarget, 0, 0, mode, servoapos, servobpos, motdangle+1000, xymode, liftmode]
    arduino.write_registers(0, registers)
    arduino.write_register(15, servocpos)

    root.update_idletasks()

    encoderdeg = arduino.read_registers(12, 3)
    print encoderdeg
    
    #encoderadeg = encoderdeg[0]/100
    #encoderbdeg = encoderdeg[1]/100
    #encoderddeg = encoderdeg[2]/100
    
    #print("Encoder A: {}, B: {}, C: {}".format(encoderadeg, encoderbdeg, encoderddeg))


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
    if(servoapos < 10):
        servoapos = 10
    servoaval.config(text=servoapos)
      
def servoaincrement(event):
    global servoapos
    servoapos += 1
    if(servoapos > 130):
        servoapos = 130
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
    if(servobpos < 0):
        servobpos = 0
    servobval.config(text=servobpos)
    
    
def servobincrement(event):
    global servobpos
    servobpos += 1
    if(servobpos > 145):
        servobpos = 145
    servobval.config(text=servobpos)
    

servoblab = Label(root, text="Servo B: ")
servoblab.grid(row=5, column=0)

root.bind("[",servobdecrement)
root.bind("]",servobincrement)

servocval = Label(root, text=servocpos)
servocval.grid(row=6, column=2)
  
def servocdecrement(event):
    global servocpos
    servocpos -= 1
    if(servocpos < 10):
        servocpos = 10
    servocval.config(text=servocpos)
      
def servocincrement(event):
    global servocpos
    servocpos += 1
    if(servocpos > 130):
        servocpos = 130
    servocval.config(text=servocpos)
    
servoclab = Label(root, text="Servo C: ")
servoclab.grid(row=6, column=0)

root.bind("-",servocdecrement)
root.bind("=",servocincrement)


motdval = Label(root, text=motdangle)
motdval.grid(row=8, column=2)

def motddecrement(event):
    global motdangle 
    motdangle -= 1
    if(motdangle < -360):
        motdangle = -360
    motdval.config(text=motdangle)
    
def motdincrement(event):
    global motdangle 
    motdangle += 1
    if(motdangle > 360):
        motdangle = 360
    motdval.config(text=motdangle)  

motdlab = Label(root, text="Motor D: ")
motdlab.grid(row=8, column=0)

root.bind(",",motddecrement)
root.bind(".",motdincrement)


def printresults(event):
    global xtarget, ytarget, bendpreference, basetarget, servoapos, servobpos, motdangle, servocpos, liftmode, xymode
    print("{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}".format(xtarget, ytarget, bendpreference, basetarget, 0, 0, mode, servoapos, servobpos, motdangle, xymode, liftmode, servocpos))


root.bind("p",printresults)

def zero():
    global xtarget, mode, motdangle, servobpos, servoapos, servocpos, basetarget, bendpreference, ytarget
    xtarget = 0
    ytarget = 249 * 2
    bendpreference = 1
    basetarget = 0
    servoapos = 110 #opposite is 45 (Fork)
    servobpos = 180 #opposite is 20 (Flip)
    servobpos = 90
    motdangle = 0
    mode = 3
    updatearduinoregisters(0)

zero = Button(root, text="Zero", command=zero, height=2, width=8, bg='navy blue', fg='white')
zero.grid(row=8, column=2)

root.bind("<Return>",updatearduinoregisters)


def runnext():
    global rawlast, xtarget, ytarget, bendpreference, basetarget, servoapos, servobpos, motdangle, xymode, liftmode, mode, servocpos

    rawlast = f.tell()
    raw = f.readline()
    wapoint = [int(x) for x in raw.split(',') if x]
    wapoint[0] += 1000
    wapoint[1] += 1000
    print wapoint
    
    xtarget = wapoint[0] - 1000
    xval.config(text=xtarget)
    ytarget = wapoint[1] - 1000
    yval.config(text=ytarget)
    bendpreference = wapoint[2]
    bendval.config(text=bendpreference)
    basetarget = wapoint[3]
    baseval.config(text=basetarget)
    servoapos = wapoint[7]
    servoaval.config(text=servoapos)
    servobpos = wapoint[8]
    servobval.config(text=servobpos)
    motdangle = wapoint[9]
    motdval.config(text=motdangle)
    xymode = wapoint[10]
    liftmode = wapoint[11]
    servocpos = wapoint[15]
    mode = 2

    updatearduinoregisters()
    root.update_idletasks()

runnextbut = Button(root, text="Next", command=runnext, height=2, width=8, bg='green')
runnextbut.grid(row=9, column=1)
    
def checksequence():
    global stopping
    if(stopping == 0):
        modelocal = arduino.read_register(6)
        if(modelocal == 0):
            time.sleep(0.1);
            runnext()
        root.after(100, checksequence)
    else:
        arduino.write_register(6, 0)
        stopping = 0

start = Button(root, text='Start', command=checksequence, height=2, width=8, bg='sky blue')
start.grid(row=9, column=3)

def stopmot():
    global stopping
    stopping = 1

stop = Button(root, text='STOP', command=stopmot, height=2, width=8, bg='red')
stop.grid(row=8, column=3)

root.mainloop()
