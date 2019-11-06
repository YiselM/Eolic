import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation
from matplotlib import style

import tkinter as tk
from tkinter import ttk         #CSS para tkinter
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
import time
import sys

LARGE_FONT= ("Verdana", 30)
MEDIUM_FONT= ("Verdana", 15)
style.use("ggplot")

f = Figure(figsize=(7,4.5), dpi=100)
a = f.add_subplot(111)

filteredVol = []
filteredCur = []

DataVoltage = []
DataCurrent = []
DataPower = []
DataOut = []
t = []

K = 0

code_enable = True


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
    a.plot(t,DataPower)
    


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
    
    
class Code_thread(threading.Thread):
    global DAC
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

    def run(self):
        global code_enable
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
         
        # Create a DAC instance.
        
        
        # Create Current and Voltage Filters
        VolFilter = IIR2Filter(2,[5],'lowpass','butter',fs=2000)
        CurFilter = IIR2Filter(2,[200],'lowpass','butter',fs=2000)
        #PIDFilter = IIR2Filter(1,[20],'lowpass','butter',fs=1000)
    #--------------------------------------------------------------------------------------------------

        
        start = time.time()

    #-----------------------------------------PID SETUP-----------------------------------------------
        #pid = PID(0.55,0.9,0.005)
        
        pid.SetPoint=20
        pid.setSampleTime(0.01)
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
        while code_enable:
            try:
                Current = ch0.voltage
                Voltage = ch3.voltage
            #-----------------------------------------IRR FILTER----------------------------------------------
                DataVoltage.append(VolFilter.filter(Voltage))
                DataCurrent.append(CurFilter.filter(Current))     
            #-------------------------------------------------------------------------------------------------
                timenow=(time.time()-start)
                t.append(timenow)
                
                if (timenow > 0 and timenow < 15):
                    pid.SetPoint=20
                elif (timenow > 15 and timenow < 30):
                    pid.SetPoint=30
                elif (timenow > 30 ):
                    pid.SetPoint=10
                    
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
                #print("| {0:^5.3f} | {1:^5.3f} | {2:^5.3f} |".format(DataCurrent[i],DataVoltage[i],DataPower[i]))
                i = i+1
            except IOError:
                print("IOError")

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
    
    def set_K(self,n):
        global K
        K = n

    def __init__(self, parent, controller):
        global pid, LKp
        tk.Frame.__init__(self,parent)
        LKp = pid.getKp()

        label = tk.Label(self, text = "Parametros de Control", font = LARGE_FONT)
        label.place(x = 450, y = 60, width = 600, height = 100)

        button1 = ttk.Button(self, text = "Atrás",
                              command = lambda: controller.show_frame(Principal))
        button1.place(x = 100, y = 65, width = 200, height = 80)

        button2 = ttk.Button(self, text = "Insertar Kp",
                              command = lambda: [controller.show_frame(Teclado),self.set_K(1)])
        button2.place(x = 250, y = 230, width = 300, height = 100)

        button3 = ttk.Button(self, text = "Insertar Ki",
                              command = lambda: [controller.show_frame(Teclado),self.set_K(2)])
        button3.place(x = 250, y = 400, width = 300, height = 100)

        button4 = ttk.Button(self, text = "Insertar Kd",
                              command = lambda: [controller.show_frame(Teclado),self.set_K(3)])
        button4.place(x = 250, y =570, width = 300, height = 100)

        label2 = tk.Label(self, bg = "white", text = str(pid.getKp()))
        label2.place(x = 600, y = 230, width = 200, height = 100)

        label3 = tk.Label(self, bg = "white", text = str(pid.getKi()))
        label3.place(x = 600, y = 400, width = 200, height = 100)

        label4 = tk.Label(self, bg = "white", text = str(pid.getKd()))
        label4.place(x = 600, y = 570, width = 200, height = 100)


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
    global code_enable
    
    def startcode(self):
        global code_enable
        try:
            self.code.start()
        except RuntimeError: 
            print('RuntimeError')
            code_enable = True
    def Setcode_enable(self, f):
        global code_enable
        code_enable=f
    
    def __init__(self, parent, controller):

        tk.Frame.__init__(self,parent)
        
        self.code = Code_thread()
        self.code.daemon = True

        #button1 = ttk.Button(self, text = "Atrás",
        #                      command = lambda: [code.kill(),DAC.set_voltage(0),controller.show_frame(Principal)], image = "")
        button1 = ttk.Button(self, text = "Atrás",
                              command = lambda: [self.Setcode_enable(False),DAC.set_voltage(0),controller.show_frame(Principal)], image = "")
        button1.place(x = 100, y = 60, width = 200, height = 80)
        
        button2 = ttk.Button(self, text = "RUN",
                              command = lambda: self.startcode(), image = "")
        button2.place(x = 1100, y = 760, width = 200, height = 80)

        label = tk.Label(self, text = "Potencia del Emulador", font = LARGE_FONT)
        label.place(x = 450, y = 60, width = 600, height = 70) 

        canvas = FigureCanvasTkAgg(f,self)
        canvas.draw()
        canvas.get_tk_widget().pack(side = "bottom", pady=200, padx = 100, fill = "x")

        label2 = tk.Label(self, text = " Nueva Consigna:", font = ("Verdana", 28))
        label2.place(x = 100, y = 760, width = 350, height = 80)

        entry = tk.Entry(self, font = ("Verdana", 28), bg = "white")
        entry.place(x = 470, y = 760, width = 200, height = 80)


class Teclado(tk.Frame):
                            
    def __init__(self, parent, controller):

        tk.Frame.__init__(self,parent)

        e = tk.Entry(self, bg = "white", font = LARGE_FONT)
        e.place(x = 900, y = 280, width = 300, height = 100)

        def button_click (number):

            digito_anterior = e.get()
            e.delete(0, tk.END)
            e.insert(0, str(digito_anterior) + str(number))

        def button_clear():

            e.delete(0, tk.END)
        
        def K_selection():
            global pid,LKp
            if K == 1:
                pid.setKp(e.get())
                print ('Kp = ' + e.get())
                
            if K == 2:
                pid.setKi(e.get())
                print ('Ki = ' + e.get())
            if K == 3:
                pid.setKd(e.get())
                print('Kd = '+ e.get())
        

        label = tk.Label(self, text = "Insertar Constante", font = LARGE_FONT)
        label.place(x = 450, y = 60, width = 600, height = 70) 

        button1 = ttk.Button(self, text = "Confirmar valor",
                              command = lambda: [controller.show_frame(Parametros), K_selection()], image = "")
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


DAC = Adafruit_MCP4725.MCP4725(address=0x60, busnum=1)
pid = PID(0.55,1,0.005)
Interfaz = Emulador_UNIGRID()
Interfaz.attributes('-zoomed', True)
ani = animation.FuncAnimation(f, animate, interval = 100)
Interfaz.mainloop()
