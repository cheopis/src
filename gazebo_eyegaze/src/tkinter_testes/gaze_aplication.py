from tkinter import *
import tkinter
import json

class Application(Frame):
    def __init__(self, master = None):
        Frame.__init__(self)
        self.parent = master
        self.initUI()

    def initUI(self):
        self.parent.title("Layout Test")
        self.config(bg = '#F0F0F0')
        self.pack(fill = BOTH, expand = 1)

        self.curBut = [-1,-1]
        self.buttonL = [[]]
        self.entry = Text(self, width=100, height=10)
        self.entry.grid(row=0, columnspan=5)
        self.createWidgets()

    def createWidgets(self):    
        but = tkinter.Button(self, text=" Prev ", width=25, bg="#000000", fg="#ffffff", highlightthickness=4, 
                        activebackground="#ffffff", activeforeground="#000000", relief="raised", padx=12,
                        pady=4, bd=4, command=lambda x=" Prev ", i=0, j=0: self.select(x, i, j))
        self.buttonL[0].append(but)
        but.grid(row=0, column=0)

        but = tkinter.Button(self, text=" Next ", width=25, bg="#000000", fg="#ffffff", highlightthickness=4, 
                        activebackground="#ffffff", activeforeground="#000000", relief="raised", padx=12,
                        pady=4, bd=4, command=lambda x=" Next ", i=0, j=1: self.select(x, i, j))
        self.buttonL[0].append(but)
        but.grid(row=0, column=1)
        self.buttonL.append([])

        but = tkinter.Button(self, text=" Select ", width=60, bg="#000000", fg="#ffffff", highlightthickness=4, 
                        activebackground="#ffffff", activeforeground="#000000", relief="raised", padx=4,
                        pady=4, bd=4, command=lambda x=" Select ", i=1, j=0: self.select(x, i, j))
        self.buttonL[1].append(but)
        but.grid(row=1, column = 0, columnspan=2, sticky = tkinter.W+tkinter.E)
        self.buttonL.append([])

        but = tkinter.Button(self, text=" Quit ", width=60, bg="#000000", fg="#ffffff", highlightthickness=4, 
                        activebackground="#ffffff", activeforeground="#000000", relief="raised", padx=4,
                        pady=4, bd=4, command = self.quit)
        self.buttonL[2].append(but)
        but.grid(row=2, column = 0, columnspan=2, sticky = tkinter.W+tkinter.E)
        self.buttonL.append([])

    def invoke_button(self,event):
        self.buttonL[self.curBut[0]][self.curBut[1]].invoke()


    def leftKey(self,event):
        # Iniciate program
        if self.curBut == [-1,-1]:
            self.curBut[:] = [0,0]
            self.buttonL[0][0].configure(highlightbackground='red')

        elif self.curBut[0] == 0:
            self.buttonL[self.curBut[0]][self.curBut[1]].configure(highlightbackground='#d9d9d9')
            self.curBut[:] = [0,0]
            self.buttonL[0][0].configure(highlightbackground='red')
        else:
            pass

    def rightKey(self,event):
        if self.curBut == [-1,-1]:
            self.curBut[:] = [0,0]
            self.buttonL[0][0].configure(highlightbackground='red')
        elif self.curBut[0] == 0:
            self.buttonL[self.curBut[0]][self.curBut[1]].configure(highlightbackground='#d9d9d9')
            self.curBut[:] = [0,1]
            self.buttonL[0][1].configure(highlightbackground='red')
        else:
            pass

    def upKey(self,event):
        if self.curBut == [-1,-1]:
            self.curBut[:] = [0,0]
            self.buttonL[0][0].configure(highlightbackground='red')
        elif self.curBut[0] == 0:
            pass
        else:
            self.buttonL[self.curBut[0]][self.curBut[1]].configure(highlightbackground='#d9d9d9')
            self.curBut[:] = [(self.curBut[0]-1), 0]
            self.buttonL[self.curBut[0]][self.curBut[1]].configure(highlightbackground='red')

    def downKey(self,event):
        if self.curBut == [-1,-1]:
            self.curBut[:] = [0,0]
            self.buttonL[0][0].configure(highlightbackground='red')
        elif self.curBut[0] == 2:
            pass
        else:
            self.buttonL[self.curBut[0]][self.curBut[1]].configure(highlightbackground='#d9d9d9')
            self.curBut[:] = [(self.curBut[0]+1), 0]
            self.buttonL[self.curBut[0]][self.curBut[1]].configure(highlightbackground='red')

    def select(self,value, x, y):
        if self.curBut != [-1,-1]:
            self.buttonL[self.curBut[0]][self.curBut[1]].configure(highlightbackground='#d9d9d9')
        self.curBut[:] = [x,y]
        self.buttonL[x][y].configure(highlightbackground='red')
        if value == "<-":
            input = self.entry.get("1.0", 'end-2c')
            self.entry.delete("1.0", END)
            self.entry.insert("1.0", input, END)

        elif value == " Space ":
            self.entry.insert(tkinter.END, ' ')

        elif value == "Tab":
            self.entry.insert(tkinter.END, '   ')

        else:
            self.entry.insert(tkinter.END, value)

def startApp():
    root = Tk()
    root.geometry('800x600+10+50')
    app = Application(root)

    # This will bind keyboard events to the tkinter
    root.bind('<Left>', lambda e: app.leftKey(e))
    root.bind('<Right>', lambda e: app.rightKey(e))
    root.bind('<Up>', lambda e: app.upKey(e))
    root.bind('<Down>', lambda e: app.downKey(e))
    root.bind('<space>', lambda e: app.invoke_button(e))

    app.mainloop() 

if __name__ == '__main__':
    startApp()   