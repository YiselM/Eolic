import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation
from matplotlib import style

import tkinter as tk
from tkinter import *
from tkinter import ttk         #CSS para tkinter
from ttkthemes import ThemedStyle
from PIL import Image, ImageTk

import os
import time
import board
import busio
import numpy as np
import matplotlib.pyplot as plt
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from IIR2Filter import IIR2Filter
import Adafruit_MCP4725

import threading
import queue
import time
import sys

import csv
from tkinter.filedialog import askopenfilename
from PIL import Image, ImageTk

LARGE_FONT= ("Verdana", 30)
MEDIUM_FONT= ("Verdana", 15)
SMALL_FONT= ("Verdana", 10)
style.use("ggplot")

f = Figure(figsize=(7,4.5), dpi=100)
plt.ylim(0,100)
a = f.add_subplot(111)



filteredVol = []
filteredCur = []

DataVoltage = []
DataCurrent = []
DataPower = []
DataOut = []
t = []

consigna = []



class PID:
    """PID Controller
    """
    def __init__(self, P=0.2, I=0.0, D=0.0, current_time=None):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.min = -400
        self.max = 400

        self.sample_time = 0.01
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        .. figure:: images/pid_1.png
           :align:   center
           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        """
        error = self.SetPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < self.min):
                self.ITerm = self.min
            elif (self.ITerm > self.max):
                self.ITerm = self.max

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain
        
    def getKp(self):
        return self.Kp

    def getKi(self):
        return self.Ki

    def getKd(self):
        return self.Kd    

    def setWindup(self, PIDmin, PIDmax):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.min = PIDmin
        self.max = PIDmax

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time


def animate(i):

    a.clear()
    a.plot(t,DataPower,'r',t,consigna,'b')
    a.set_ylim(0,70)
    a.set_xlim(left=0)
    


class Emulador_UNIGRID(tk.Tk):
    
    def __init__(self, *args, **kwargs):
        
        tk.Tk.__init__(self,*args,**kwargs)
        self.wm_title(' ')
        container = ttk.Frame(self)
        style = ThemedStyle()
        style.set_theme('radiance')
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight = 1)
        container.grid_columnconfigure(0, weight = 1)
        self.frames = {}
        self.protocol("WM_DELETE_WINDOW", self.on_exit)

        for F in (Principal, Parametros, Perfiles, Visual, Teclado):
            frame = F(container,self)
            
            #frame.config(bg='snow')
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame(Principal)

    def show_frame(self, cont):
        frame = self.frames[cont]
        frame.tkraise()
    def on_exit(self):
        global DAC
        """When you click to exit, this function is called"""
        #if messagebox.askyesno("Exit", "Realmente quieres salir de Emulador Unigrid?"):        
        self.destroy()
        #DAC.set_voltage(0)
        sys.exit()
    
class Code_thread(threading.Thread):
        
        def __init__(self):
            threading.Thread.__init__(self)
            self.killed = False 
        
        def start(self): 
            self.__run_backup = self.run 
            self.run = self.__run       
            threading.Thread.start(self) 
      
        def __run(self): 
            sys.settrace(self.globaltrace) 
            self.__run_backup() 
            self.run = self.__run_backup 
          
        def globaltrace(self, frame, event, arg): 
            if event == 'call': 
              return self.localtrace 
            else: 
              return None
          
        def localtrace(self, frame, event, arg): 
            if self.killed: 
              if event == 'line': 
                raise SystemExit() 
            return self.localtrace 
          
        def kill(self): 
            self.killed = True
        
        def seti(self,i):
            self.i = i   

        def run(self):
            global DAC, WPt, WPpw, mode
                        
        #---------------------------------Inicializacion-----------------------------------------------------
            # Create the I2C bus
            i2c = busio.I2C(board.SCL, board.SDA)
            
            # Create the ADC object using the I2C bus
            ads = ADS.ADS1115(i2c)

            # Create single-ended input on channel 0
            ch0 = AnalogIn(ads, ADS.P0)
            ch3 = AnalogIn(ads, ADS.P3)
            ch2 = AnalogIn(ads, ADS.P2)

            # Create differential input between channel 0 and 1
            dif01 = AnalogIn(ads, ADS.P0, ADS.P1)
            dif23 = AnalogIn(ads, ADS.P2, ADS.P3)

            ads.gain = 2/3
            ads.data_rate = 860
            
            # Create Current and Voltage Filters
            VolFilter = IIR2Filter(2,[5],'lowpass','butter',fs=1000)
            CurFilter = IIR2Filter(2,[200],'lowpass','butter',fs=1000)
        #--------------------------------------------------------------------------------------------------

            start = time.time()

        #-----------------------------------------PID SETUP-----------------------------------------------
            #pid = PID(0.55,0.9,0.005)
            pid = PID(0.55,1,0.01)
            pid.SetPoint=20
            pid.setSampleTime(0.001)
            feedback = 0
            feedback_list = []
            time_list = []

            pidmin = 0 
            pidmax = 5
        # -----------------------------------------------------------------------------------------------

            voltajedac = 0
            DAC.set_voltage(voltajedac)
            i=0;
        
        #------------------------------------- MAIN LOOP--------------------------------------------------    
            for i in range(500):
                
                try:
                    Current = ch0.voltage
                    Voltage = ch3.voltage
                #-----------------------------------------IRR FILTER----------------------------------------------
                    DataVoltage.append(VolFilter.filter(Voltage))
                    DataCurrent.append(CurFilter.filter(Current))     
                #-------------------------------------------------------------------------------------------------
                #-------------------------------------------------------------------------------------------------
                    timenow=(time.time()-start)
                    t.append(timenow)
                    mode=q.get()
                    # if mode == 'Test':                
                        # if (timenow > 0 and timenow < 15):
                            # pid.SetPoint=20
                        # elif (timenow > 15 and timenow < 30):
                            # pid.SetPoint=30
                        # elif (timenow > 30 ):
                            # pid.SetPoint=10
                        # q.put('Test')
                    # elif mode == 'Perfil':
                        # for j in range(len(WPt)-1):
                            # if (timenow > WPt[j] and timenow < WPt[j+1]):
                                # pid.SetPoint=WPpw[j]
                        # q.put('Perfil')
                    pid.SetPoint = 10
                #--------------------------------Para graficar la consigna-----------------------------------------        
                    if i==0:
                        consigna.append(0)
                    else:
                        consigna.append(pid.SetPoint)
                #-------------------------------------------------------------------------------------------------
                    DataVoltage[i]=DataVoltage[i]*9.5853-0.1082
                    DataCurrent[i]=DataCurrent[i]*1.4089+0.1326
                    
                    DataPower.append(DataVoltage[i]*DataCurrent[i])
                # --------------------------------------- PID CONTROLLER------------------------------------------
                    pid.update(DataPower[i])
                    output = pid.output
                    
                    if pid.SetPoint > 0:
                        voltajedac = voltajedac + (output - (1/(i+1)))
                    
                    if voltajedac < pidmin:
                        voltajedac = pidmin
                    elif voltajedac > pidmax:
                        voltajedac = pidmax
                # ---------------------------------------------DAC------------------------------------------------
                    voltbits=int((4096/5)*voltajedac)
                    DAC.set_voltage(voltbits)    
                  
                # ------------------------------------------------------------------------------------------------   
                    #i = i+1
                except IOError:
                    print('IOError')


class Principal(tk.Frame):

    def __init__(self, parent, controller):
        
        
        tk.Frame.__init__(self,parent)
        
        
        
        label = ttk.Label(self, text = "Emulador Eolico UNIGRID", font = LARGE_FONT, style = 'TButton')
        label.place(x = 450, y = 60, width = 600, height = 100)
        
        button1 = ttk.Button(self, text = "Parametros de Control",
                              command = lambda: [self.configLabelParametros(), self.setLabelParametros(), controller.show_frame(Parametros)]
                                    , style = 'Style.TButton')
        button1.place(x = 200, y = 250, width = 400, height = 100)

        button2 = ttk.Button(self, text = "Cargar Perfiles de Viento",
                              command = lambda: controller.show_frame(Perfiles)
                                    , style = 'Style.TButton')
        button2.place(x = 200, y = 450, width = 400, height = 100)

        button3 = ttk.Button(self, text = "Iniciar",
                              command = lambda: controller.show_frame(Visual)
                                    , style = 'Style.TButton')
        button3.place(x = 900, y =600, width = 300, height = 100)
        
    def configLabelParametros(self):
        global KpLabel, KiLabel, KdLabel,label2,label3,label4
        label2.config(textvariable=KpLabel)
        label3.config(textvariable=KiLabel)
        label4.config(textvariable=KdLabel)
        
    def setLabelParametros(self):
        global KpLabel, KiLabel, KdLabel,label2,label3,label4
        # pid = qpid.get()
        # qpid.set(pid)
        KpLabel.set(str(pid.getKp()))
        KiLabel.set(str(pid.getKi()))
        KdLabel.set(str(pid.getKd()))

       
class Parametros(tk.Frame):
    def set_K(self,n):
        global K
        K = n
    def __init__(self, parent, controller):
        global pid, KpLabel, KiLabel, KdLabel,label2,label3,label4
        tk.Frame.__init__(self,parent)
        
        # pid = qpid.get()
        # qpid.set(pid)

        label = tk.Label(self, text = "Parametros de Control", font = LARGE_FONT)
        label.place(x = 450, y = 60, width = 600, height = 100)

        button1 = ttk.Button(self, text = "Atrás",
                              command = lambda: controller.show_frame(Principal))
        button1.place(x = 100, y = 65, width = 200, height = 80)

        button2 = ttk.Button(self, text = "Insertar Kp",
                              command = lambda: [controller.show_frame(Teclado),self.set_K(1)], image = "")
        button2.place(x = 250, y = 230, width = 300, height = 100)

        button3 = ttk.Button(self, text = "Insertar Ki",
                              command = lambda: [controller.show_frame(Teclado),self.set_K(2)], image = "")
        button3.place(x = 250, y = 400, width = 300, height = 100)

        button4 = ttk.Button(self, text = "Insertar Kd",
                              command = lambda: [controller.show_frame(Teclado),self.set_K(3)], image = "")
        button4.place(x = 250, y =570, width = 300, height = 100)
        
        button5 = ttk.Button(self, text = "Confirmar PID",
                              command = lambda: '', image = "")
        button5.place(x = 1000, y =400, width = 200, height = 200)

        label2 = tk.Label(self, bg = "white", text = str(pid.getKp()))
        label2.place(x = 600, y = 230, width = 200, height = 100)

        label3 = tk.Label(self, bg = "white", text = str(pid.getKi()))
        label3.place(x = 600, y = 400, width = 200, height = 100)

        label4 = tk.Label(self, bg = "white", text = str(pid.getKd()))
        label4.place(x = 600, y = 570, width = 200, height = 100)



class Perfiles(tk.Frame):
    def OpenWindFile(self):
        global WPt, WPpw
        tkopenfile = Tk()
        tkopenfile.withdraw()
        filename = askopenfilename()
        tkopenfile.destroy()
        WPt, WPpw = self.readcsv(filename)
        
    def readcsv(self,filename):
        global q, textvarOpenFile
        try:
            ifile = open(filename, "r")
            reader = csv.reader(ifile, delimiter=" ")
            rownum = 0
            x = []
            y = []
            for row in reader:
                col0 = row[0]
                hh,mm = col0.split(":")
                col0 = int(hh)*3600+int(mm)*60
                col0 = col0/3600*10
                col1 = float(row[1].replace(",","."))
                col1 = (0.001723483*(col1**6)-0.04935507*(col1**5)+0.01124858*(col1**4)
                        +12.34628*(col1**3)-144.3604*(col1**2)+657.3997*col1-1038.827)*(1/10)
                print(str(col0)+" "+str(col1))
                x.append(col0)
                y.append(col1)        
                rownum += 1    
            ifile.close()
            filesplit = filename.split("/")
            namefile = filesplit[-1]
            textvarOpenFile.set('El archivo '+namefile+' ha sido cargado')
            mode = 'Perfil'
            q.put('Perfil')
            return x,y
        except (TypeError,FileNotFoundError) as e:
            print('Archivo no abierto')
    def __init__(self, parent, controller):
        global textvarOpenFile, labelFileOk
        tk.Frame.__init__(self,parent)
        label = tk.Label(self, text = "Perfiles de Viento", font = LARGE_FONT)
        label.place(x = 450, y = 60, width = 600, height = 100) 
        
        button1 = ttk.Button(self, text = "Atrás",
                              command = lambda: controller.show_frame(Principal))
        button1.place(x = 100, y = 65, width = 200, height = 80)
        
        button2 = ttk.Button(self, text = "Abrir",
                              command = lambda: self.OpenWindFile(), image = "")
        button2.place(x = 400, y = 300, width = 200, height = 200)
        
        labelFileOk = tk.Label(self, text = '' ,font = SMALL_FONT)
        labelFileOk.place(x = 400, y = 550, width = 500, height = 50)


class Visual(tk.Frame):
    def startCodeThread(self):
        try:
            self.code.start()
        except RuntimeError:
            self.cleargraph()
            self.code = Code_thread()
            self.code.daemon = True
            self.code.start()
    def cleargraph(self):
        a.clear()
        filteredVol.clear()
        filteredCur.clear()
        DataVoltage.clear()
        DataCurrent.clear()
        DataPower.clear()
        t.clear()
        consigna.clear()
        self.code.seti(0)
    def __init__(self, parent, controller):
        global DAC,bttstop,bttgo,bttback
        tk.Frame.__init__(self,parent)
        
        self.code = Code_thread()
        self.code.daemon = True     
       
        
        bttback = ttk.Button(self, text = "Atrás",
                              command = lambda: [controller.show_frame(Principal)], image = "")
        bttback.place(x = 50, y = 30, width = 100, height = 40)
        
        bttgo = ttk.Button(self, text = "Start",
                              command = lambda: self.startCodeThread(), image = "")
        bttgo.place(x = 700, y = 680, width = 70, height = 70)
        
        bttstop = ttk.Button(self, text = "Stop",
                              command = lambda: [DAC.set_voltage(0),print('presiono stop'),self.code.kill()], image = '')
        bttstop.place(x = 800, y = 680, width = 70, height = 70)
               
        canvas = FigureCanvasTkAgg(f,self)
        canvas.draw()
        canvas.get_tk_widget().pack(side = "bottom", pady=200, padx = 100, fill = "x")
        


class Teclado(tk.Frame):
                                    
    def __init__(self, parent, controller):

        tk.Frame.__init__(self,parent)

        self.e = tk.Entry(self, bg = "white", font = LARGE_FONT)
        self.e.place(x = 900, y = 280, width = 300, height = 100)

        def button_click (number):

            digito_anterior = self.e.get()
            self.e.delete(0, tk.END)
            self.e.insert(0, str(digito_anterior) + str(number))

        def button_clear():

            self.e.delete(0, tk.END)
        
        def button_clear():

            self.e.delete(0, tk.END)
            
        def K_selection():
            global KpLabel, KiLabel, KdLabel    
                
            if K == 1:
                #pid.setKp(e.get())
                print ('Kp = ' + self.e.get())
                KpLabel.set(self.e.get())
                
            if K == 2:
                #pid.setKi(e.get())
                print ('Ki = ' + self.e.get())
                KiLabel.set(self.e.get())
            if K == 3:
                #pid.setKd(e.get())
                print('Kd = '+ self.e.get())
                KdLabel.set(self.e.get())

        label = tk.Label(self, text = "Insertar Constante", font = LARGE_FONT)
        label.place(x = 450, y = 60, width = 600, height = 70) 

        button1 = ttk.Button(self, text = "Confirmar valor",
                              command = lambda: [controller.show_frame(Parametros), K_selection(),self.e.delete(0,len(self.e.get()))], image = "")
        button1.place(x = 900, y =600, width = 300, height = 100)

        button2 = ttk.Button(self, text = "Atrás",
                              command = lambda: controller.show_frame(Principal))
        button2.place(x = 100, y = 70, width = 200, height = 80)
        
        button3 = ttk.Button(self, text = "7",
                              command = lambda: button_click(7))
        button3.place(x = 250, y = 280, width = 100, height = 100)
        
        button4 = ttk.Button(self, text = "8",
                              command = lambda: button_click(8))
        button4.place(x = 350, y = 280, width = 100, height = 100) 
        
        button4 = ttk.Button(self, text = "9",
                              command = lambda: button_click(9))
        button4.place(x = 450, y = 280, width = 100, height = 100)  
        
        button5 = ttk.Button(self, text = "4",
                              command = lambda: button_click(4))
        button5.place(x = 250, y = 380, width = 100, height = 100) 
        
        button6 = ttk.Button(self, text = "5",
                              command = lambda: button_click(5))
        button6.place(x = 350, y = 380, width = 100, height = 100)
        
        button7 = ttk.Button(self, text = "6",
                              command = lambda: button_click(6))
        button7.place(x = 450, y = 380, width = 100, height = 100)  
        
        button8 = ttk.Button(self, text = "1",
                              command = lambda: button_click(1))
        button8.place(x = 250, y = 480, width = 100, height = 100) 
        
        button9 = ttk.Button(self, text = "2",
                              command = lambda: button_click(2))
        button9.place(x = 350, y = 480, width = 100, height = 100) 
        
        button10 = ttk.Button(self, text = "3",
                              command = lambda: button_click(3))
        button10.place(x = 450, y = 480, width = 100, height = 100)
        
        button11 = ttk.Button(self, text = ".",
                              command = lambda: button_click('.'))
        button11.place(x = 250, y = 580, width = 100, height = 100)
        
        button11 = ttk.Button(self, text = "0",
                              command = lambda: button_click(0))
        button11.place(x = 350, y = 580, width = 100, height = 100)  
        
        button11 = ttk.Button(self, text = "borrar",
                              command = button_clear)
        button11.place(x = 450, y = 580, width = 100, height = 100) 


#--------------Create a DAC instance------------------
DAC = Adafruit_MCP4725.MCP4725(address=0x60, busnum=1)
#-----------Pila para el modo del emulador------------
q=queue.LifoQueue()
mode = 'Test'
q.put(mode)
#-------------------------PID-------------------------
pid = PID(0.55,1,0.01)
#--------------------Clase root Tk()------------------
Interfaz = Emulador_UNIGRID()
#Interfaz.configure(bg = 'snow')
#---------------------ThemedStyle---------------------------
# style = ThemedStyle(Interfaz)
# style.set_theme('radiance')
# Interfaz.style.theme_use('radiance')
# Interfaz.style.configure('Style.TButton', font = ('helvetica',12), background = 'red', foreground = 'snow')
#-------------------------Style-----------------------------
style = ttk.Style()
style.configure("TButton", padding=6, relief="flat",
                    background="snow")
#---------Añadir iconos a botones---------------------
imag = Image.open("stopimg.png")
stopimg = ImageTk.PhotoImage(imag,master = Interfaz)
bttstop.configure(image=stopimg)
imag = Image.open("goimg.png")
goimg = ImageTk.PhotoImage(imag,master = Interfaz)
bttgo.configure(image=goimg)
#---------Label de los parametros del PID-------------
KpLabel = StringVar(Interfaz)
KiLabel = StringVar(Interfaz)
KdLabel = StringVar(Interfaz)
KpLabel.set(pid.getKp())
KiLabel.set(pid.getKi())
KdLabel.set(pid.getKd())
#-----------------------------------------------------
textvarOpenFile = StringVar(Interfaz)
textvarOpenFile.set('')
labelFileOk.configure(textvariable = textvarOpenFile)
#-----------------------------------------------------
Interfaz.attributes('-zoomed', True)
ani = animation.FuncAnimation(f, animate, interval = 100)
Interfaz.mainloop()
