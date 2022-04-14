from Tkinter import Tk, Canvas, Frame, Button
from Tkinter import BOTH, W, NW, SUNKEN, TOP, X, FLAT, LEFT

class Example(Frame):
    def __init__(self, parent):
        Frame.__init__(self, parent)
        self.parent = parent
        self.initUI()

    def initUI(self):
        self.parent.title("Layout Test")
        self.config(bg = '#F0F0F0')
        self.pack(fill = BOTH, expand = 1)
                #create canvas
        canvas1 = Canvas(self, relief = FLAT, background = "#D2D2D2", width = 180, height = 500)
        canvas1.pack(side = TOP, anchor = NW, padx = 10, pady = 10)
        
        #add quit button
        button1 = Button(self, text = "Quit", command = self.quit, anchor = W)
        button1.configure(width = 10, activebackground = "#33B5E5", relief = FLAT)
        button1_window = canvas1.create_window(10, 10, anchor=NW, window=button1)
        button1.pack(side = TOP)

def main():
    root = Tk()
    root.geometry('800x600+10+50')
    app = Example(root)
    app.mainloop()

if __name__ == '__main__':
    main()