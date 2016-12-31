# Arduino Modbus Register Table
# Register:     Register Name:      Source:         Description:
#   0               Side                Slave          Penny side (0=empty, 1=heads, 2=tails)
from Tkinter import *
import minimalmodbus
from pymodbus.server.async import StartTcpServer
from pymodbus.server.async import StartUdpServer
from pymodbus.server.async import StartSerialServer

from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer

import logging
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)

root = Tk()

def heads():
    0x00.setValue(0, 0x00, 1)
def tails():
    0x00.setValue(0, 0x00, 2)
def empty():
    0x00.setValue(0, 0x00, 0)
    
bheads = Button(root, text="Heads", command=heads)
btails = Button(root, text="Tails", command=tails)
bempty = Button(root, text="Empty", command=empty)

bheads.grid(row=0, column=0)
btails.grid(row=0, column=2)
bempty.grid(row=0, column=1)

store = ModbusSlaveContext(hr = ModbusSequentialDataBlock(0, [0]*100))
context = ModbusServerContext(slaves=store, single=True)

StartTcpServer(context, address=("192.168.5.1", 502))

store = ModbusSlaveContext()

root.mainloop()
