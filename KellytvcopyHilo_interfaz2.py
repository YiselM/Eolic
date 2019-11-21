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
mode = 'Test'


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
        
        def __init__(self):
            threading.Thread.__init__(self)
            self.i=0 
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
            global DAC, mode
                        
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

            
            starttime = time.time()

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
            
        #------------------------------------- MAIN LOOP--------------------------------------------------    
            while True:
                try:
                    print('aqui')
                    Current = ch0.voltage
                    Voltage = ch3.voltage
                #-----------------------------------------IRR FILTER----------------------------------------------
                    DataVoltage.append(VolFilter.filter(Voltage))
                    DataCurrent.append(CurFilter.filter(Current))     
                #-------------------------------------------------------------------------------------------------
                    timenow=(time.time()-starttime)
                    t.append(timenow)
                    if mode == 'Test':
                        if (timenow > 0 and timenow < 15):
                            pid.SetPoint=20
                        elif (timenow > 15 and timenow < 30):
                            pid.SetPoint=30
                        elif (timenow > 30 ):
                            pid.SetPoint=10
                    consigna.append(pid.SetPoint)
                    DataVoltage[self.i]=DataVoltage[self.i]*9.5853-0.1082
                    DataCurrent[self.i]=DataCurrent[self.i]*1.4089+0.1326
                    
                    DataPower.append(DataVoltage[self.i]*DataCurrent[self.i])
                # --------------------------------------- PID CONTROLLER------------------------------------------
                    pid.update(DataPower[self.i])
                    output = pid.output
                    
                    if pid.SetPoint > 0:
                        voltajedac = voltajedac + (output - (1/(self.i+1)))
                    
                    if voltajedac < pidmin:
                        voltajedac = pidmin
                    elif voltajedac > pidmax:
                        voltajedac = pidmax
                # ---------------------------------------------DAC------------------------------------------------
                    voltbits=int((4096/5)*voltajedac)
                    DAC.set_voltage(voltbits)    
                  
                # ------------------------------------------------------------------------------------------------   
                    #print("| {0:^5.3f} | {1:^5.3f} | {2:^5.3f} |".format(DataCurrent[i],DataVoltage[i],DataPower[i]))
                    self.i = self.i+1
                except IOError:
                    print('IOError')


class Principal(tk.Frame):

    def __init__(self, parent, controller):

        tk.Frame.__init__(self,parent)

        label = tk.Label(self, text = "Emulador Eolico UNIGRID", font = LARGE_FONT)
        label.place(x = 225, y = 60, width = 300, height = 500)


        button1 = ttk.Button(self, text = "Parametros de Control",
                              command = lambda: controller.show_frame(Parametros))
        button1.place(x = 100, y = 125, width = 200, height = 50)

        button2 = ttk.Button(self, text = "Cargar Perfiles de Viento",
                              command = lambda: controller.show_frame(Perfiles))
        button2.place(x = 100, y = 225, width = 200, height = 50)

        button3 = ttk.Button(self, text = "Iniciar",
                              command = lambda: controller.show_frame(Visual))
        button3.place(x = 350, y =200, width = 150, height = 50)

       
class Parametros(tk.Frame):

    def __init__(self, parent, controller):

        tk.Frame.__init__(self,parent)


        label = tk.Label(self, text = "Parametros de Control", font = LARGE_FONT)
        label.place(x = 225, y = 30, width = 300, height = 50)

        button1 = ttk.Button(self, text = "Atrás",
                              command = lambda: controller.show_frame(Principal))
        button1.place(x = 50, y = 32, width = 100, height = 40)

        button2 = ttk.Button(self, text = "Insertar Kp",
                              command = lambda: controller.show_frame(Teclado), image = "")
        button2.place(x = 125, y = 115, width = 150, height = 50)

        button3 = ttk.Button(self, text = "Insertar Ki",
                              command = lambda: controller.show_frame(Teclado), image = "")
        button3.place(x = 125, y = 200, width = 150, height = 50)

        button4 = ttk.Button(self, text = "Insertar Kd",
                              command = lambda: controller.show_frame(Teclado), image = "")
        button4.place(x = 125, y =285, width = 150, height = 50)

        entry = tk.Entry(self, bg = "white", text = "kp")
        entry.place(x = 300, y = 115, width = 100, height = 50)

        entry2 = tk.Entry(self, bg = "white", text = "ki")
        entry2.place(x = 300, y = 200, width = 100, height = 50)

        entry3 = tk.Entry(self, bg = "white", text = "kd")
        entry3.place(x = 300, y = 285, width = 100, height = 50)


class Perfiles(tk.Frame):

    def __init__(self, parent, controller):

        tk.Frame.__init__(self,parent)

        label = tk.Label(self, text = "Perfiles de Viento", font = LARGE_FONT)
        label.place(x = 225, y = 30, width = 300, height = 50) 

        label2 = tk.Label(self, text = "Fuente", anchor = "w", padx = 200, font = MEDIUM_FONT, bg = "red", fg = "white")
        label2.place(x = 0, y = 100, width = 680, height = 25) 

        button1 = ttk.Button(self, text = "Atrás",
                              command = lambda: controller.show_frame(Principal))
        button1.place(x = 50, y = 32, width = 100, height = 40)


class Visual(tk.Frame):
    
    def cleargraph(self):
        a.clear()
        filteredVol.clear()
        filteredCur.clear()
        DataVoltage.clear()
        DataCurrent.clear()
        DataPower.clear()
        t.clear()
        code.seti(0)    

    def __init__(self, parent, controller):
        global DAC
        tk.Frame.__init__(self,parent)
        
        self.code = Code_thread()
        self.code.daemon = True        

        button1 = ttk.Button(self, text = "Atrás",
                              command = lambda: [DAC.set_voltage(0),controller.show_frame(Principal)], image = "")
        button1.place(x = 50, y = 30, width = 100, height = 40)
        
        button2 = ttk.Button(self, text = "Start",
                              command = lambda: [self.code.start()], image = "")
        button2.place(x = 400, y = 30, width = 100, height = 40)
        
        button3 = ttk.Button(self, text = "Stop",
                              command = [print('presionó stop'),DAC.set_voltage(0),self.code.kill()], image = "")
        button3.place(x = 550, y = 30, width = 100, height = 40)
        
        canvas = FigureCanvasTkAgg(f,self)
        canvas.draw()
        canvas.get_tk_widget().pack(side = "bottom", pady=100, padx = 50, fill = "x")

        


class Teclado(tk.Frame):
                                    
    def __init__(self, parent, controller):

        tk.Frame.__init__(self,parent)

        e = tk.Entry(self, bg = "white", font = LARGE_FONT)
        e.place(x = 450, y = 140, width = 150, height = 50)

        def button_click (number):

            digito_anterior = e.get()
            e.delete(0, tk.END)
            e.insert(0, str(digito_anterior) + str(number))

        def button_clear():

            e.delete(0, tk.END)


        label = tk.Label(self, text = "Insertar Constante", font = LARGE_FONT)
        label.place(x = 225, y = 30, width = 300, height = 35) 

        button1 = ttk.Button(self, text = "Confirmar valor",
                              command = lambda: controller.show_frame(Parametros), image = "")
        button1.place(x = 450, y =300, width = 150, height = 50)

        button2 = ttk.Button(self, text = "Atrás",
                              command = lambda: controller.show_frame(Principal))
        button2.place(x = 50, y = 35, width = 100, height = 40)
        
        button3 = ttk.Button(self, text = "7",
                              command = lambda: button_click(7))
        button3.place(x = 125, y = 140, width = 50, height = 50)
        
        button4 = ttk.Button(self, text = "8",
                              command = lambda: button_click(8))
        button4.place(x = 175, y = 140, width = 50, height = 50) 
        
        button4 = ttk.Button(self, text = "9",
                              command = lambda: button_click(9))
        button4.place(x = 225, y = 140, width = 50, height = 50)  
        
        button5 = ttk.Button(self, text = "4",
                              command = lambda: button_click(4))
        button5.place(x = 125, y = 190, width = 50, height = 50) 
        
        button6 = ttk.Button(self, text = "5",
                              command = lambda: button_click(5))
        button6.place(x = 175, y = 190, width = 50, height = 50)
        
        button7 = ttk.Button(self, text = "6",
                              command = lambda: button_click(6))
        button7.place(x = 225, y = 190, width = 50, height = 50)  
        
        button8 = ttk.Button(self, text = "1",
                              command = lambda: button_click(1))
        button8.place(x = 125, y = 240, width = 50, height = 50) 
        
        button9 = ttk.Button(self, text = "2",
                              command = lambda: button_click(2))
        button9.place(x = 175, y = 240, width = 50, height = 50) 
        
        button10 = ttk.Button(self, text = "3",
                              command = lambda: button_click(3))
        button10.place(x = 225, y = 240, width = 50, height = 50)
        
        button11 = ttk.Button(self, text = ".",
                              command = lambda: button_click('.'))
        button11.place(x = 125, y = 290, width = 50, height = 50)
        
        button11 = ttk.Button(self, text = "0",
                              command = lambda: button_click(0))
        button11.place(x = 175, y = 290, width = 50, height = 50)  
        
        button11 = ttk.Button(self, text = "borrar",
                              command = button_clear)
        button11.place(x = 225, y = 290, width = 50, height = 50) 


# Create a DAC instance.
DAC = Adafruit_MCP4725.MCP4725(address=0x60, busnum=1)
Interfaz = Emulador_UNIGRID()
#Interfaz.attributes('-zoomed', True)
#Interfaz.geometry('1024x500')
ani = animation.FuncAnimation(f, animate, interval = 100)
Interfaz.mainloop()
