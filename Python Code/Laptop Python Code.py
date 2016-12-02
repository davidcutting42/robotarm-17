# Arduino Modbus Register Table
# Register:     Register Name:      Source:         Description:
#   0               Side                Master          Penny side (0=empty, 1=heads, 2=tails)
from Tkinter import *
import minimalmodbus

root = Tk()

def heads():
    print("heads")
def tails():
    print("tails")
def empty():
    print("empty")
    
bheads = Button(root, text="Heads", command=heads)
btails = Button(root, text="Tails", command=tails)
bempty = Button(root, text="Empty", command=empty)

bheads.grid(row=0, column=0)
btails.grid(row=0, column=2)
bempty.grid(row=0, column=1)

root.mainloop()
