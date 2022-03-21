import Tkinter as tk
main=tk.Tk()
main.title("grid() method")
main.geometry("350x275")

tableFrame = tk.Frame(main)
tableFrame.pack()


tableFrame.grid(column=0,row=0, sticky="nsew")

main.mainloop()