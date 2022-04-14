#!/usr/bin/env python2

#app libraries
from tkinter import *
from tkinter.ttk import *
from Tkinter import Tk, Canvas, Frame, Button
from Tkinter import BOTH, W, NW, SUNKEN, TOP, X, FLAT, LEFT


global m
m = PyMouse()

class Application(Frame):
    def __init__(self, master = None):
        Frame.__init__(self)
        self.parent = master
        self.initUI()
        
    def initUI(self):
        self.parent.title("Layout Test")
        self.config(bg = '#F0F0F0')
        self.pack(fill = BOTH, expand = 1)

        #create canvas
        self.canvas = Canvas(self, relief = FLAT, background = "#D2D2D2", width = 180, height = 500)
        self.canvas.pack(side = TOP, anchor = NW, padx = 10, pady = 10)

        #add gazebo camera image
        #### FAZER ISSO AQUI QUANDO TUDO DER BOM (O( ####
        
        #create buttons
        self.createWidgets()

    # eyegaze mouse functions
    def leftClick(self, event):
        x0,y0 = m.position()
        m.click(x0,y0,1)
	
    def rightClick(self, event):
        x0,y0 = m.position()
        m.click(x0,y0,0)

    def holdClick(self, event):
        x0,y0 = m.position()
        m.press(x0,y0)
        m.release(x0,y0)

    # select the item and send the information to robot
    def selectItem(self,item):
        print("selected: ", item)

    # creating the buttons
    def createWidgets(self):
        self.QUIT = Button(self, bg="red", text = "Quit", command = self.quit)
        self.QUIT.configure(width = 10, activebackground = "#33B5E5", anchor=W, relief = FLAT)
        #button_window = self.canvas.create_window(10, 10, anchor=NW, window=self.QUIT)
        self.QUIT.place(x=10, y=10)
        

        self.banana = Button(self, text = "Select Item", command = self.selectItem('banana'))
        self.banana.configure(width = 10, activebackground = "#33B5E5", anchor=W, relief = FLAT)
        #button_window = self.canvas.create_window(0, 0, anchor=NW, window=self.hi_there)
        self.banana.place(x=10, y=50)

def startApp():
    root = Tk()
    root.geometry('800x600+10+50')
    app = Application(root)

    # This will bind eyegaze events to the tkinter
    # toplevel which will define the mouse clicks
    root.bind("<KeyPress-Left>", lambda e: app.leftClick(e))
    root.bind("<KeyPress-Right>", lambda e: app.rightClick(e))
    root.bind("<space>", lambda e: app.holdClick(e))

    app.mainloop() 

if __name__ == '__main__':
    startApp()   
