import Tkinter as tkinter

master=tkinter.Tk()
master.title("grid() method")
master.geometry("350x275")

button1=tkinter.Button(master,text=u"\u00f7")
button1.grid(row=1,column=0)

master.mainloop()