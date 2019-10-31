import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation
from matplotlib import style

import tkinter as tk
from tkinter import ttk         #CSS para tkinter
from PIL import Image, ImageTk



LARGE_FONT= ("Verdana", 30)
MEDIUM_FONT= ("Verdana", 15)
style.use("ggplot")

f = Figure(figsize=(7,4.5), dpi=100)
a = f.add_subplot(111)


def animate(i):

    pullData = open("sampleData.txt","r").read()
    dataList = pullData.split("\n")
    xList = []
    yList = []
   
    for eachLine in dataList:

        if len(eachLine) > 1:
            x, y = eachLine.split(",")
            xList.append(int(x))
            yList.append(int(y))
    a.clear()
    a.plot(xList, yList)


class Emulador_UNIGRID(tk.Tk):
    
    def __init__(self, *args, **kwargs):
        
        tk.Tk.__init__(self,*args,**kwargs)

        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight = 1)
        container.grid_columnconfigure(0, weight = 1)

        self.frames = {}

        for F in (Principal, Parametros, Perfiles, Visual, Teclado):
            frame = F(container,self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame(Principal)

    def show_frame(self, cont):
        frame = self.frames[cont]
        frame.tkraise()


class Principal(tk.Frame):

    def __init__(self, parent, controller):

        tk.Frame.__init__(self,parent)

        label = tk.Label(self, text = "Emulador Eolico UNIGRID", font = LARGE_FONT)
        label.place(x = 450, y = 60, width = 600, height = 100)


        button1 = ttk.Button(self, text = "Parametros de Control",
                              command = lambda: controller.show_frame(Parametros))
        button1.place(x = 200, y = 250, width = 400, height = 100)

        button2 = ttk.Button(self, text = "Cargar Perfiles de Viento",
                              command = lambda: controller.show_frame(Perfiles))
        button2.place(x = 200, y = 450, width = 400, height = 100)

        button3 = ttk.Button(self, text = "Iniciar",
                              command = lambda: controller.show_frame(Visual))
        button3.place(x = 900, y =600, width = 300, height = 100)

       


class Parametros(tk.Frame):

    def __init__(self, parent, controller):

        tk.Frame.__init__(self,parent)

        label = tk.Label(self, text = "Parametros de Control", font = LARGE_FONT)
        label.place(x = 450, y = 60, width = 600, height = 100)

        button1 = ttk.Button(self, text = "Atrás",
                              command = lambda: controller.show_frame(Principal))
        button1.place(x = 100, y = 65, width = 200, height = 80)

        button2 = ttk.Button(self, text = "Insertar Kp",
                              command = lambda: controller.show_frame(Teclado), image = "")
        button2.place(x = 250, y = 230, width = 300, height = 100)

        button3 = ttk.Button(self, text = "Insertar Ki",
                              command = lambda: controller.show_frame(Teclado), image = "")
        button3.place(x = 250, y = 400, width = 300, height = 100)

        button4 = ttk.Button(self, text = "Insertar Kd",
                              command = lambda: controller.show_frame(Teclado), image = "")
        button4.place(x = 250, y =570, width = 300, height = 100)



class Perfiles(tk.Frame):

    def __init__(self, parent, controller):

        tk.Frame.__init__(self,parent)

        label = tk.Label(self, text = "Perfiles de Viento", font = LARGE_FONT)
        label.place(x = 450, y = 60, width = 600, height = 100) 

        label2 = tk.Label(self, text = "Fuente", anchor = "w", padx = 200, font = MEDIUM_FONT, bg = "red", fg = "white")
        label2.place(x = 0, y = 200, width = 1360, height = 100) 

        button1 = ttk.Button(self, text = "Atrás",
                              command = lambda: controller.show_frame(Principal))
        button1.place(x = 100, y = 65, width = 200, height = 80)


class Visual(tk.Frame):

    def __init__(self, parent, controller):

        tk.Frame.__init__(self,parent)

        button1 = ttk.Button(self, text = "Atrás",
                              command = lambda: controller.show_frame(Principal), image = "")
        button1.place(x = 100, y = 70, width = 200, height = 80)

        label = tk.Label(self, text = "Potencia del Emulador", font = LARGE_FONT)
        label.place(x = 450, y = 60, width = 600, height = 70) 

        label2 = tk.Label(self, text = "Potencia: ")
        label2.place(x = 1000, y =400, width = 50, height = 50)

        label3 = tk.Label(self, text = "Watts")
        label3.place(x = 1050, y =400, width = 50, height = 50)

        canvas = FigureCanvasTkAgg(f,self)
        canvas.draw()
        canvas.get_tk_widget().pack(side = "left", pady=200, padx=100)





class Teclado(tk.Frame):
                                    
    def __init__(self, parent, controller):

        tk.Frame.__init__(self,parent)

        label = tk.Label(self, text = "Insertar Constante", font = LARGE_FONT)
        label.grid(row = 0, column= 2, pady = 10, padx = 30, sticky = "n") 

        button1 = ttk.Button(self, text = "Confirmar valor",
                              command = lambda: controller.show_frame(Parametros), image = "")
        button1.grid(row = 4, column= 6, pady = 10, padx = 10, sticky = "e")

        button2 = ttk.Button(self, text = "Atrás",
                              command = lambda: controller.show_frame(Principal))
        button2.grid(row = 0, column= 0, pady = 10, padx = 10, sticky = "w")





Interfaz = Emulador_UNIGRID()
Interfaz.attributes('-fullscreen', True)
ani = animation.FuncAnimation(f, animate, interval = 100)
Interfaz.mainloop()