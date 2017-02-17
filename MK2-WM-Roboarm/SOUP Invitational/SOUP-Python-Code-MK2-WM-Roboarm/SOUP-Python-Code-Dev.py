from Tkinter import *

root = Tk()

xtarget = 0
ytarget = 0

class incdec():
    'Common class for all tkinter motor interface portions controlled by keystrokes'
    def __init__(self, label, value, decbutton, incbutton, incdecres, labelcol, labelrow, valuecol, valuerow):
        global root
        self.value = value
        self.incdecres = incdecres
        self.valdisp = Label(root, text=self.value)
        self.valdisp.grid(row=valuerow, column=valuecol)
        self.labdisp = Label(root, text=label)
        self.labdisp.grid(row=labelrow, column=labelcol)
        root.bind_all(decbutton, self.decrement)
        root.bind_all(incbutton, self.increment)
        root.bind_all(printbut, self.printval)
    def increment(self, event):
        self.value += self.incdecres
        self.valdisp.config(text=self.value)
    def decrement(self, event):
        self.value -= self.incdecres
        self.valdisp.config(text=self.value)

xincdec = incdec("X:",  xtarget, "<Left>", "<Right>", 1, 0, 0, 1, 0)
yincdec = incdec("Y:", ytarget, "<Up>", "<Down>", 1, 0, 1, 1, 1)

root.mainloop()
