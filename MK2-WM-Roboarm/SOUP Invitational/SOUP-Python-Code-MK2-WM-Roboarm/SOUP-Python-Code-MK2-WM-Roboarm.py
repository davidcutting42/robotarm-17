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

def updatearduinoregisters(event): #Function for stepper control and spacenav input
    global xtarget, ytarget, bendpreference, basetarget, servoapos, servobpos, motdangle, xymode, liftmode, mode, servocpos

    registers = [xtarget+1000, ytarget+1000, bendpreference, basetarget, 0, 0, mode, servoapos, servobpos, motdangle+1000, xymode, liftmode]
    arduino.write_registers(0, registers)
    arduino.write_register(15, servocpos)

    root.update_idletasks()

class incdec():
    'Common class for all tkinter motor interface portions controlled by keystrokes'
    def __init__(self, label, value, decbutton, incbutton, incdecres, labelcol, labelrow, valuecol, valuerow, minimum, maximum):
        global root
        self.minimum = minimum
        self.maximum = maximum
        self.value = value
        self.incdecres = incdecres
        self.valdisp = Label(root, text=self.value)
        self.valdisp.grid(row=valuerow, column=valuecol)
        self.labdisp = Label(root, text=label)
        self.labdisp.grid(row=labelrow, column=labelcol)
        root.bind_all(decbutton, self.decrement)
        root.bind_all(incbutton, self.increment)
    def increment(self, event):
        self.value += self.incdecres
        self.valdisp.config(text=self.value)
        if(self.value < self.minimum):
            self.value = self.minimum
        if(self.value > self.maximum):
            self.value = self.maximum
        updatearduinoregisters(0)
    def decrement(self, event):
        self.value -= self.incdecres
        self.valdisp.config(text=self.value)
        if(self.value < self.minimum):
            self.value = self.minimum
        if(self.value > self.maximum):
            self.value = self.maximum
        updatearduinoregisters(0)

xincdec = incdec("X:", xtarget, "<Left>", "<Right>", 1, 0, 0, 1, 0, -10000, 10000)
yincdec = incdec("Y:", ytarget, "<Up>", "<Down>", 1, 0, 1, 1, 1, -10000, 10000)
bendval = incdec("Bend:", bendpreference, "<F1>", "<F2>", 1, 0, 2, 1, 2, 0, 1)
baseval = incdec("Base:", basetarget, "<Shift-Up>", "<Shift-Down>", 1, 0, 3, 1, 3, 0, 95)
servoaval = incdec("Servo A (Fork):", servoapos, "9", "0", 5, 0, 4, 1, 4, 0, 180)
servobval = incdec("Servo B (Flip):", servobpos, "-", "=", 5, 0, 5, 1, 5, 0, 180)
servocval = incdec("Servo C (Lift):", servocpos, "[", "]", 5, 0, 6, 1, 6, 0, 180)
motdval = incdec("Stepper D:", motdangle, ",", ".", 3, 0, 7, 1, 7, 0, 360)

def printresults(event):
    global xtarget, ytarget, bendpreference, basetarget, servoapos, servobpos, motdangle, servocpos, liftmode, xymode
    print("{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}".format(xtarget, ytarget, bendpreference, basetarget, 0, 0, mode, servoapos, servobpos, motdangle, xymode, liftmode, servocpos))

def zero(event):
    global xtarget, mode, motdangle, servobpos, servoapos, servocpos, basetarget, bendpreference, ytarget
    xtarget = 0
    ytarget = 249 * 2
    bendpreference = 1
    basetarget = 0
    servoapos = 0 #opposite is 45 (Fork)
    servobpos = 180 #opposite is 20 (Flip)
    servobpos = 90
    motdangle = 0
    mode = 3
    updatearduinoregisters(0)

def runnext(event):
    global rawlast, xtarget, ytarget, bendpreference, basetarget, servoapos, servobpos, motdangle, xymode, liftmode, mode, servocpos

    rawlast = f.tell()
    raw = f.readline()
    wapoint = [int(x) for x in raw.split(',') if x]
    wapoint[0] += 1000
    wapoint[1] += 1000
    print wapoint
    
    xtarget = wapoint[0] - 1000
    xval.config(text=xincdec.value)
    ytarget = wapoint[1] - 1000
    yval.config(text=yincdec.value)
    bendpreference = wapoint[2]
    bendval.config(text=bendpreference.value)
    basetarget = wapoint[3]
    baseval.config(text=basetarget.value)
    servoapos = wapoint[7]
    servoaval.config(text=servoapos.value)
    servobpos = wapoint[8]
    servobval.config(text=servobpos.value)
    motdangle = wapoint[9]
    motdval.config(text=motdangle.value)
    xymode = wapoint[10]
    liftmode = wapoint[11]
    servocpos = wapoint[12]
    mode = 2

    updatearduinoregisters(0)
    root.update_idletasks()
    
def checksequence(event):
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

def stopmot(event):
    global stopping
    stopping = 1

root.bind("<Tab>", runnext)
root.bind("p",printresults)
root.bind("z", zero)
root.bind("<Shift-Tab>", checksequence)
root.bind("<space>", stopmot)
root.bind("<Return>", updatearduinoregisters)

root.mainloop()
