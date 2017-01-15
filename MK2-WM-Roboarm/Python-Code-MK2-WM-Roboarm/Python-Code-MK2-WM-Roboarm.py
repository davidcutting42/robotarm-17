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

# Wait 2 Seconds for all communications channels to come on board.
time.sleep(2)

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

def updatearduinoregisters(): #Function for stepper control and spacenav input
    registers = [xtarget+1000, ytarget+1000, bendpreference, basetarget, 0, 0, mode, servoapos, servobpos, motdangle]
    #print registers
    arduino.write_registers(0, registers)
    root.update_idletasks()
    
def xdecrement():
    global xtarget
    xtarget -= 1
    xval.config(text=xtarget)
    updatearduinoregisters()
    
def xincrement():
    global xtarget
    xtarget += 1
    xval.config(text=xtarget)
    updatearduinoregisters()
    

xlab = Label(root, text="X: ")
xlab.grid(row=0, column=0)
xinc = Button(root, text="<", command=xdecrement, height=3, width=8, bg='orange red', repeatdelay=500, repeatinterval=100)
xinc.grid(row=0, column=1)
xval = Label(root, text=xtarget)
xval.grid(row=0, column=2)
xinc = Button(root, text=">", command=xincrement, height=3, width=8, bg='orange red', repeatdelay=500, repeatinterval=100)
xinc.grid(row=0, column=3)

    
def ydecrement():
    global ytarget
    ytarget -= 1
    yval.config(text=ytarget)
    updatearduinoregisters()
    
def yincrement():
    global ytarget
    ytarget += 1
    yval.config(text=ytarget)
    updatearduinoregisters()
    
ylab = Label(root, text="Y: ")
ylab.grid(row=1, column=0)
yinc = Button(root, text="<", command=ydecrement, height=3, width=8, bg='orange red', repeatdelay=500, repeatinterval=100)
yinc.grid(row=1, column=1)
yval = Label(root, text=ytarget)
yval.grid(row=1, column=2)
yinc = Button(root, text=">", command=yincrement, height=3, width=8, bg='orange red', repeatdelay=500, repeatinterval=100)
yinc.grid(row=1, column=3)

    
def benddecrement():
    global bendpreference
    bendpreference = 0
    bendval.config(text=bendpreference)
    updatearduinoregisters()
    
def bendincrement():
    global bendpreference
    bendpreference = 1
    bendval.config(text=bendpreference)
    updatearduinoregisters()
    

bendlab = Label(root, text="Bend: ")
bendlab.grid(row=2, column=0)
bendinc = Button(root, text="<", command=benddecrement, height=3, width=8, bg='white')
bendinc.grid(row=2, column=1)
bendval = Label(root, text=bendpreference)
bendval.grid(row=2, column=2)
bendinc = Button(root, text=">", command=bendincrement, height=3, width=8, bg='white')
bendinc.grid(row=2, column=3)
    
def basedecrement():
    global basetarget
    basetarget -= 1 
    baseval.config(text=basetarget)
    updatearduinoregisters()
    
def baseincrement():
    global basetarget
    basetarget += 1
    baseval.config(text=basetarget)
    updatearduinoregisters()
    

baselab = Label(root, text="Base: ")
baselab.grid(row=3, column=0)
baseinc = Button(root, text="<", command=basedecrement, height=3, width=8, bg='orange red', repeatdelay=500, repeatinterval=100)
baseinc.grid(row=3, column=1)
baseval = Label(root, text=basetarget)
baseval.grid(row=3, column=2)
baseinc = Button(root, text=">", command=baseincrement, height=3, width=8, bg='orange red', repeatdelay=500, repeatinterval=100)
baseinc.grid(row=3, column=3)
    
def servoadecrement():
    global servoapos
    servoapos -= 1
    servoaval.config(text=servoapos)
    updatearduinoregisters()
    
def servoaincrement():
    global servoapos
    servoapos += 1
    servoaval.config(text=servoapos)
    updatearduinoregisters()
    

servoalab = Label(root, text="Servo A: ")
servoalab.grid(row=4, column=0)
servoainc = Button(root, text="<", command=servoadecrement, height=3, width=8, bg='orange red', repeatdelay=500, repeatinterval=100)
servoainc.grid(row=4, column=1)
servoaval = Label(root, text=servoapos)
servoaval.grid(row=4, column=2)
servoainc = Button(root, text=">", command=servoaincrement, height=3, width=8, bg='orange red', repeatdelay=500, repeatinterval=100)
servoainc.grid(row=4, column=3)
    
def servobdecrement():
    global servobpos
    servobpos -= 1
    servobval.config(text=servobpos)
    updatearduinoregisters()
    
def servobincrement():
    global servobpos
    servobpos += 1
    servobval.config(text=servobpos)
    updatearduinoregisters()
    

servoblab = Label(root, text="Servo B: ")
servoblab.grid(row=5, column=0)
servobinc = Button(root, text="<", command=servobdecrement, height=3, width=8, bg='orange red', repeatdelay=500, repeatinterval=100)
servobinc.grid(row=5, column=1)
servobval = Label(root, text=servobpos)
servobval.grid(row=5, column=2)
servobinc = Button(root, text=">", command=servobincrement, height=3, width=8, bg='orange red', repeatdelay=500, repeatinterval=100)
servobinc.grid(row=5, column=3)

def motddecrement():
    global motdangle 
    motdangle -= 1
    motdval.config(text=motdangle)
    updatearduinoregisters()
    
def motdincrement():
    global motdangle 
    motdangle += 1
    motdval.config(text=motdangle)
    updatearduinoregisters()
    

motdlab = Label(root, text="Motor D: ")
motdlab.grid(row=6, column=0)
motdinc = Button(root, text="<", command=motddecrement, height=3, width=8, bg='orange red', repeatdelay=500, repeatinterval=100)
motdinc.grid(row=6, column=1)
motdval = Label(root, text=motdangle)
motdval.grid(row=6, column=2)
motdinc = Button(root, text=">", command=motdincrement, height=3, width=8, bg='orange red', repeatdelay=500, repeatinterval=100)
motdinc.grid(row=6, column=3)

def printresults():
    global xtarget
    global ytarget
    global bendpreference
    global basetarget
    global servoapos
    global servobpos
    global motdangle
    print "{}, {}, {}, {}, {}, {}, {}, 0, 0, 0".format(xtarget, ytarget, bendpreference, basetarget, motdangle, servoapos, servobpos)

printresults = Button(root, text="Print", command=printresults, height=3, width=8, bg='green')
printresults.grid(row=7, column=2)

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
    updatearduinoregisters()
    mode = 2

zero = Button(root, text="Zero", command=zero, height=3, width=8, bg='red')
zero.grid(row=7, column=3)


#root.after(1, updateSteppers)
root.mainloop()
