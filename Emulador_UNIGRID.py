#!/usr/bin/env python
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

import random

LARGE_FONT= ("Verdana", 35,'bold')
LARGE_FONT2= ("Verdana", 30)
MEDIUM_FONT= ("Verdana", 10)
SMALL_FONT= ("Verdana", 4)

ColorTitulo = '#DE0002'
style.use("ggplot")

f = Figure(figsize=(12,7.7), dpi=100, facecolor= "snow")

plt.ylim(0,100)
a = f.add_subplot(111)
pos1=a.get_position()
pos=[pos1.x0,pos1.y0-0.05,pos1.width,pos1.height-0.05]
a.set_position(pos)
       


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
    try:
        a.clear()
        a.plot(t,DataPower,'r',t,consigna,'b')
        a.set_ylim(0,60)
        a.set_xlim(left=0) 
        a.set_xlabel('Time (s)')
        a.set_ylabel('Delivered Power (W)')
        a.legend(['Power','Setpoint'])
      
    except ValueError:
        a.clear()
        a.plot(t,DataPower,'r',t,consigna,'b')
        a.set_ylim(0,60)
        a.set_xlim(left=0) 
        a.set_xlabel('Time (s)')
        a.set_ylabel('Delivered Power (W)')
        a.legend(['Power','Setpoint'])
        pass
def set_K(n):
    global K
    K = n

class Emulador_UNIGRID(tk.Tk):
    
    def __init__(self, *args, **kwargs):
        
        tk.Tk.__init__(self,*args,**kwargs)
        self.wm_title('UniGRID WT Emulator')
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight = 1)
        container.grid_columnconfigure(0, weight = 1)
        self.frames = {}
        self.protocol("WM_DELETE_WINDOW", self.on_exit)
        
        self.icono = tk.PhotoImage(file='/home/pi/Desktop/Eolic_Emulator_Control/eolicimg.gif', master=self)
        self.tk.call('wm','iconphoto',self._w,self.icono)

        for F in (Principal, Parametros, Perfiles, Visual, Teclado):
            frame = F(container,self)
            frame.config(bg='snow')
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame(Visual)

    def show_frame(self, cont):
        frame = self.frames[cont]
        frame.tkraise()
    def on_exit(self):
        global DAC
        """When you click to exit, this function is called"""
        if messagebox.askyesno("Exit", "Are you sure do you want to exit the emulator console?"):        
            self.destroy()
            DAC.set_voltage(0)
            sys.exit()
              
        
class ShowPower(threading.Thread):
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
        while True:
            if (qpower.empty()==False):
                actpow.set(str(round(qpower.get(),2))+" W")
    
    
class Code_thread(threading.Thread):
        
        def __init__(self):
            threading.Thread.__init__(self)
            self.powermeter = ShowPower()
            self.powermeter.daemon = True
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
            global npid
            npid.set('')
            self.powermeter.kill() 
            self.killed = True
        
        def seti(self,i):
            self.i = i 
            
        def writecsv(self,filename,x,y,z):
            ifile = open(filename,mode='w')
            writer = csv.writer(ifile,delimiter = ' ')
            for i in range(len(x)):
                xx = str(x[i]).replace('.',',')
                yy = str(y[i]).replace('.',',')
                zz = str(z[i]).replace('.',',')
                writer.writerow([xx,yy,zz])
                print(xx+" "+yy+" "+zz)
            ifile.close()

        def run(self):
            global DAC, WPt, WPpw, mode, actpow, qsetpoint
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
            VolFilter = IIR2Filter(1,[10],'lowpass','cheby2',fs=5000)
            CurFilter = IIR2Filter(1,[10],'lowpass','cheby2',fs=5000)
        #--------------------------------------------------------------------------------------------------


        #-----------------------------------------PID SETUP-----------------------------------------------
            #pid = PID(0.55,0.9,0.005)
            #pid = PID(0.55,1,0.01)
            if (qkp.empty()==False):
                kp = qkp.get()
                qkp.put(kp)
            else:
                kp = 0.01
            if (qki.empty()==False):
                ki = qki.get()
                qki.put(ki)
            else:
                ki = 0.00005
            if (qkd.empty()==False):
                kd = qkd.get()
                qkd.put(kd)
            else:
                kd = 0.004
            pid = PID(kp,ki,kd)
            npid.set("PID = {0}, {1}, {2}".format(str(pid.getKp()),str(pid.getKi()),str(pid.getKd())))
            print('PID = {0},{1},{2}'.format(pid.getKp(),pid.getKi(),pid.getKd()))
          #  time.sleep(1)
        #--------------------------------------------------------------------------------------------------
            
            pid.SetPoint=10
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
            c1 = 0
            c2 = 0
            c3 = 0
            c4 = 0
            iniciando.set('')
            starttime = time.time()
            self.powermeter.start()
        #------------------------------------- MAIN LOOP--------------------------------------------------    
            #for i in range(2000):
            while True:
                
                try:
                    Current = ch0.voltage
                    Voltage = ch3.voltage
                #-----------------------------------------IRR FILTER----------------------------------------------
                    DataVoltage.append(VolFilter.filter(Voltage))
                    DataCurrent.append(CurFilter.filter(Current))     
                #-------------------------------------------Tiempo------------------------------------------------
                    timenow=(time.time()-starttime)
                    t.append(timenow)
                #-------------------------------------------------------------------------------------------------
                    mode=q.get()
                    if mode == 'Test':                
                        if (timenow > 0 and timenow < 20):
                            pid.SetPoint=10
                        elif (timenow > 20 and timenow < 35):
                            pid.SetPoint=30
                        elif (timenow > 35 and timenow < 50):
                            pid.SetPoint=10
                        elif (timenow > 50 and timenow < 65):
                            pid.SetPoint=30
                        elif (timenow > 65 and timenow < 80):
                            pid.SetPoint=45
                        elif (timenow > 80 and timenow < 95):
                            pid.SetPoint=35
                        elif (timenow > 95 and timenow < 110):
                            pid.SetPoint=20
                        elif (timenow > 110 and timenow < 125):
                            pid.SetPoint=10
                        elif (timenow > 125 and timenow < 140):
                            pid.SetPoint=5
                        q.put('Test')
                    elif mode == 'Perfil':
                        for j in range(len(WPt)-1):
                            if (timenow > WPt[j]*Ad and timenow < WPt[j+1]*Ad):
                                pid.SetPoint=WPpw[j]
                        q.put('Perfil')
                    elif mode == 'Manual':
                        if(qsetpoint.empty()==False):
                            pid.SetPoint=float(qsetpoint.get())
                        q.put('Manual')
                #------------------------------Prueba aleatoria---------------------------------------------------
                    
                    # if (timenow > 0 and timenow < 10):
                        # c1=c1+1
                        # if(c1 == 1):
                            # rand = round(random.uniform(10,35),2)
                            # pid.SetPoint = rand
                    # elif (timenow > 10 and timenow < 20):
                        # c2=c2+1
                        # if(c2 == 1):
                            # rand = round(random.uniform(10,35),2)
                            # pid.SetPoint = rand
                    # elif (timenow > 20 and timenow < 30):
                        # c3=c3+1
                        # if(c3 == 1):
                            # rand = round(random.uniform(10,35),2)
                            # pid.SetPoint = rand
                    # elif (timenow > 30 and timenow < 40):
                        # c4=c4+1
                        # if(c4 == 1):
                            # rand = round(random.uniform(10,35),2)
                            # pid.SetPoint = rand
                    # elif (timenow > 40):
                        # t.pop()
                        # break
                #--------------------------------Para graficar la consigna-----------------------------------------        
                    consigna.append(pid.SetPoint)
                #-------------------------------------------------------------------------------------------------
                    DataVoltage[i]=DataVoltage[i]*9.5853-0.1082
                    DataCurrent[i]=DataCurrent[i]*1.4089+0.1326
                    
                    DataPower.append(DataVoltage[i]*DataCurrent[i])
                    qpower.put(DataPower[i])

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
                    i = i+1
                except IOError:
                    print('IOError')
           # self.writecsv("/media/pi/KELLY/PruebaAleatoria1_1.csv",t,DataPower,consigna)


class Principal(tk.Frame):

    def __init__(self, parent, controller):
        
        global DAC
        tk.Frame.__init__(self,parent)
        voltajedac = 3
        voltbits=int((4096/5)*voltajedac)
        DAC.set_voltage(voltbits) 
        
     
        label = tk.Label(self, text = "UNIGRID WT Emulator Console", font = MEDIUM_FONT, background ='snow', foreground = ColorTitulo)
        label.place(x =100, y=50, width=800, height=100)
        
        button1 = tk.Button(self, text = "Controller Parameters", font= MEDIUM_FONT, background ='snow',relief = 'solid',
                              command = lambda: [self.configLabelParametros(),self.setLabelParametros(), controller.show_frame(Parametros)]
                                    , )
        button1.place(x = 200, y = 250, width = 400, height = 100)

        button2 = tk.Button(self, text = "Load Wind Profiles", font= MEDIUM_FONT, background ='snow',relief = 'solid',
                              command = lambda: controller.show_frame(Perfiles))
        button2.place(x = 200, y = 450, width = 400, height = 100)

        button3 = tk.Button(self, text = "Plot", font= MEDIUM_FONT,background ='snow',relief = 'solid',
                              command = lambda: controller.show_frame(Visual))
        button3.place(x = 900, y =320, width = 200, height = 180)
        
        
        
        
        
    def configLabelParametros(self):
        global KpLabel, KiLabel, KdLabel,label2,label3,label4, Adlabel
        label1.config(textvariable=AdLabel)
        label2.config(textvariable=KpLabel)
        label3.config(textvariable=KiLabel)
        label4.config(textvariable=KdLabel)
        
    def setLabelParametros(self):
        global KpLabel, KiLabel, KdLabel,label2,label3,label4,Adlabel
      #  Ad = qad.get()
       # qad.put(Ad)
   
        kp = qkp.get()
        qkp.put(kp)
        ki = qki.get()
        qki.put(ki)
        kd = qkd.get()
        qkd.put(kd)
      #  Adlabel.set(Ad)
        KpLabel.set(kp)
        KiLabel.set(ki)
        KdLabel.set(kd)
       
class Parametros(tk.Frame):
    
    def confirmarpid(self):
        global actpid
        Kp = float(KpLabel.get())
        Ki = float(KiLabel.get())
        Kd = float(KdLabel.get())
        qkp.put(Kp)
        qki.put(Ki)
        qkd.put(Kd)
        KpLabel.set(Kp)
        KiLabel.set(Ki)
        KdLabel.set(Kd)
        print('Updated PID Controller')
        actpid.set('Updated PID')
        sleep(3)
        actpid.set('')
        
    def __init__(self, parent, controller):
        global pid, KpLabel, KiLabel, KdLabel,label2,label3,label4,labelactpid, actpid, Adlabel
        tk.Frame.__init__(self,parent)
        

        label = tk.Label(self, text = "Controller Parameters", font = MEDIUM_FONT, background='snow', foreground = ColorTitulo)
        label.place(x = 350, y = 60, width = 900, height = 100)

        button1 = tk.Button(self, text = "Back", font= MEDIUM_FONT,relief = 'solid', background='snow', 
                              command = lambda: [actpid.set(''),controller.show_frame(Principal)])
        button1.place(x = 100, y = 65, width = 200, height = 80)

        button2 = tk.Button(self, text = " Kp ",font= MEDIUM_FONT,relief = 'solid',background='snow', 
                              command = lambda: [controller.show_frame(Teclado),set_K(1)])
        button2.place(x = 250, y = 230, width = 300, height = 100)

        button3 = tk.Button(self, text = " Ki ",font= MEDIUM_FONT,relief = 'solid',background='snow', 
                              command = lambda: [controller.show_frame(Teclado),set_K(2)])
        button3.place(x = 250, y = 400, width = 300, height = 100)

        button4 = tk.Button(self, text = " Kd ",font= MEDIUM_FONT,relief = 'solid', background='snow', 
                              command = lambda: [controller.show_frame(Teclado),set_K(3)], image = "")
        button4.place(x = 250, y =570, width = 300, height = 100)
        
        button5 = tk.Button(self, text = " Ad ",font= MEDIUM_FONT,relief = 'solid', background='snow', 
                              command = lambda: [controller.show_frame(Teclado),set_K(5)], image = "")
        button5.place(x = 250, y =470, width = 300, height = 100)
      
      #  button5 = tk.Button(self, text = "Confirm PID",font= MEDIUM_FONT,relief = 'solid', background='snow', 
      #                        command = lambda:[self.confirmarpid(),actpid.set(''),controller.show_frame(Visual)], image = "")
     #   button5.place(x = 1000, y =350, width = 220, height = 200)
        

class Perfiles(tk.Frame):
    
            
    def __init__(self, parent, controller):
        global textvarOpenFile, labelFileOk
        tk.Frame.__init__(self,parent)
        label = tk.Label(self, text = "Wind Profiles", font = LARGE_FONT, background ='snow', foreground = ColorTitulo)
        label.place(x = 430, y = 60, width = 650, height = 100)
        
        button1 = tk.Button(self, text = "Back",font= MEDIUM_FONT,relief = 'solid', background='snow',
                              command = lambda: controller.show_frame(Principal))
        button1.place(x = 100, y = 65, width = 200, height = 80)
        
        button2 = tk.Button(self, text = "Open",font= MEDIUM_FONT,relief = 'solid', background='snow',
                              command = lambda: self.OpenWindFile())
        button2.place(x = 600, y = 300, width = 200, height = 200)
       
        
class Visual(tk.Frame):
    
    def startCodeThread(self):
        global iniciando, actpid, Ad 
        try:
            Ad = float(Adlabel.get())
            Kp = float(KpLabel.get())
            Ki = float(KiLabel.get())
            Kd = float(KdLabel.get())
            print(Ad)
        #    qad.put(Ad)
            qkp.put(Kp)
            qki.put(Ki)
            qkd.put(Kd)
        #    Adlabel.set(Ad)
            KpLabel.set(Kp)
            KiLabel.set(Ki)
            KdLabel.set(Kd)
            iniciando.set('Starting...')
            voltajedac = 3
            voltbits=int((4096/5)*voltajedac)
            DAC.set_voltage(voltbits)
          
            self.code.start()
            
        except RuntimeError:
            print("error2")
            self.code.kill()
            actpow.set('')
            self.cleargraph()
            self.clearqpower()
            self.code = Code_thread()
            self.code.daemon = True
            self.startCodeThread()
            
    def clearqpower(self):
        global qpower
        with qpower.mutex:
            qpower.queue.clear()
            
    def cleargraph(self):
        a.clear()
        filteredVol.clear()
        filteredCur.clear()
        DataVoltage.clear()
        DataCurrent.clear()
        DataPower.clear()
        t.clear()
        consigna.clear()
        actpow.set('')
        self.code.seti(0)
        self.clearqpower()
    def OpenWindFile(self):
        global WPt, WPpw
        try:
            tkopenfile = Tk()
            tkopenfile.withdraw()
            filename = askopenfilename(initialdir = '/media/pi')
            tkopenfile.destroy()
            print (filename)
            type(filename).__name__
            if filename != "":           
                WPt, WPpw = self.readcsv(filename)
           # else:
            #    textvarOpenFile.set('Wind Profile Load cancelled')
        except TypeError:
            pass
    def readcsv(self,filename):
        global q, textvarOpenFile
        try:
            ifile = open(filename, "r")
            reader = csv.reader(ifile, delimiter=" ")
            rownum = 0
            x = []
            y = []
            for row in reader:
                col0 = float(row[0])
                #hh,mm = col0.split(":")
                #col0 = int(hh)*3600+int(mm)*60 # Convierte hora en formato hh:mm a segundos
                #col0 = col0/3600*10 # Una hora se representa en 10 s
                #col1 = float(row[1].replace(",","."))
                
                col1 = float(row[1])
                # Ajuste en el nivel de viento
                col1 = 1.2*col1
                
                # Recta del perfil de viento (Convierte viento a potencia) y escalado al nivel del emulador (10:1)
                col1 = (0.001723483*(col1**6)-0.04935507*(col1**5)+0.01124858*(col1**4)
                        +12.34628*(col1**3)-144.3604*(col1**2)+657.3997*col1-1038.827)*(1/10)
                
                # Limites de saturación de la turbina eolica
                if col1 < 0:
                    col1 = 0
                elif col1 > 50:
                    col1 = 0
                    
                print(str(col0)+" "+str(col1))
                
                x.append(col0)
                y.append(col1)        
                rownum += 1    
            ifile.close()
            filesplit = filename.split("/")
            namefile = filesplit[-1]
          #  textvarOpenFile.set('The File '+namefile+' has been loaded')
            mode = 'Perfil'
            q.put('Perfil')
            return x,y
        except (ValueError,TypeError,FileNotFoundError) as e:
         #   textvarOpenFile.set('Load File Error')
            print('File not opened')
    def confirmarpid(self):
        global actpid
        Kp = float(KpLabel.get())
        Ki = float(KiLabel.get())
        Kd = float(KdLabel.get())
        qkp.put(Kp)
        qki.put(Ki)
        qkd.put(Kd)
        KpLabel.set(Kp)
        KiLabel.set(Ki)
        KdLabel.set(Kd)
        print('Updated PID Controller')
        actpid.set('Updated PID')
        sleep(3)
        actpid.set('')
    
    def __init__(self, parent, controller):
        global DAC,bttstop,bttgo,bttback,labelini, labelpow, actpow,labelFileOk
        global pid, KpLabel, KiLabel, KdLabel,label2,label3,label4,labelactpid, actpid, lpid, labelUni,colimg, Adlabel, label1
        tk.Frame.__init__(self,parent)
        
        self.code = Code_thread()
        self.code.daemon = True
        
        canvas = FigureCanvasTkAgg(f,self)
        canvas.draw()
        canvas.get_tk_widget().pack(side = "left")
        
        b = 150
                
        label = tk.Label(self, text = "UniGRID WT Emulator Console", font = LARGE_FONT, background ='snow', foreground = ColorTitulo)
        label.place(x = 150, y = 120, width = 1000, height = 70)
        
        labelUni = tk.Label(self,text='Uni', background ='snow')
        labelUni.place(x =20, y=5, width=390, height=100)
        
        colimg = tk.Label(self,text='Col', background ='snow')
        colimg.place(x =450, y=-10, width=390, height=110)
        
        labelnames = tk.Label(self,text='Electrical and Electronics Engineering Department',font= ("Verdana", 10,'bold'), background='snow')
        labelnames.place(x =1034, y=30, width=400, height=20)
        labelases = tk.Label(self,text='Research Group in Robotics and Intelligent Systems (GiRSi)',font= MEDIUM_FONT, background='snow')
        labelases.place(x =980, y=50, width=500, height=20)
        labelyear = tk.Label(self,text='@2019',font= MEDIUM_FONT, background='snow')
        labelyear.place(x =1080, y=70, width=300, height=20)
                
        bttgo = tk.Button(self, text = "Start",borderwidth = 0,relief = 'solid', background='snow',
                              command = lambda: [actpid.set(''),self.startCodeThread()], image = "")
        bttgo.place(x = 1130, y = 700, width = 110, height = 110)
        
        bttstop = tk.Button(self, text = "Stop",borderwidth = 0,relief = 'solid', background='snow',
                              command = lambda: [print(' stop '),self.clearqpower(),actpow.set(''),self.code.kill(),
                                    DAC.set_voltage(0),actpid.set('')], image = '')
        bttstop.place(x = 1280, y = 700, width = 110, height = 110)
               
        labelpw = tk.Label(self, text = "Power:", font =("Verdana", 18,'bold') , background ='snow')
        labelpw.place(x = 1140, y = 450+b, width = 90, height = 50)
        
      #  labelad = tk.Label(self, text = "XX:", font =("Verdana", 18,'bold') , background ='snow')
       # labelad.place(x = 1140, y = 360, width = 90, height = 50)
        
        labelpow = tk.Label(self, text = "50 W", font = ("Verdana", 18,'bold'), background ='snow')
        labelpow.place(x = 1250, y = 450+b, width = 150, height = 50)
        
        buttonconsig = tk.Button(self, text = "Manual Setpoint", font= ("Verdana", 10,'bold'),relief = 'solid', background='snow',
                                command = lambda: [controller.show_frame(Teclado),set_K(4)])
        buttonconsig.place(x = 1130, y = 230, width = 250, height = 50)
        
        buttonperfil = tk.Button(self, text = "Wind Profile",font= ("Verdana", 10,'bold'),relief = 'solid', background='snow',
                              command = lambda: self.OpenWindFile())
        buttonperfil.place(x = 1130, y = 290, width = 250, height = 50)
        
      #  labelFileOk = tk.Label(self, text = 'File' ,font= MEDIUM_FONT, background='snow')
     #   labelFileOk.place(x = 700, y = 330, width = 720, height = 50)
                
        labelini = tk.Label(self, text = "Starting...", font = ("Verdana", 14), background ='snow')
        labelini.place(x = 1200, y = 813, width = 100, height = 20)
        
        labelnamepid = tk.Label(self, text = 'PID Controller Parameters', font = ("Verdana", 10,'bold'), background='snow')
        labelnamepid.place(x = 1130, y = 270+b, width = 220, height = 30)
                
        button2 = tk.Button(self, text = " Kp ",font= ("Verdana", 10),relief = 'solid',background='snow', 
                              command = lambda: [controller.show_frame(Teclado),set_K(1)])
        button2.place(x = 1130, y = 310+b, width = 120, height = 30)

        button3 = tk.Button(self, text = " Ki ",font= ("Verdana", 10),relief = 'solid',background='snow', 
                               command = lambda: [controller.show_frame(Teclado),set_K(2)])
        button3.place(x = 1130, y = 350+b, width = 120, height = 30)

        button4 = tk.Button(self, text = " Kd ",font= ("Verdana", 10),relief = 'solid', background='snow', 
                               command = lambda: [controller.show_frame(Teclado),set_K(3)], image = "")
        button4.place(x = 1130, y = 390+b, width = 120, height = 30)
        
        button5 = tk.Button(self, text = " Time Base ",font= ("Verdana", 10),relief = 'solid', background='snow', 
                               command = lambda: [controller.show_frame(Teclado),set_K(5)], image = "")
        button5.place(x = 1130, y = 220+b, width = 120, height = 30)
        
     #   button5 = tk.Button(self, text = "Confirm PID",font= ("Verdana", 10,'bold'),relief = 'solid', background='snow', 
     #                        command = lambda:[self.confirmarpid(),actpid.set(''),controller.show_frame(Visual)], image = "")
     #   button5.place(x = 1160, y =435+b, width = 150, height = 40)
        
      #  labelactpid = tk.Label(self, text = " ", font = ("Verdana", 14), background='snow')
      #  labelactpid.place(x = 1140, y = 500+b, width = 200, height = 22)
        
     #   lpid = tk.Label(self, text = '', font = ("Verdana", 14), background='snow')
     #   lpid.place(x = 1130, y = 480+b, width = 200, height = 22)
        
        label1 = tk.Label(self, bg = "white",font= ("Verdana", 14), text = str(3))
        label1.place(x = 1265, y = 220+b, width = 80, height = 30)
        
        label2 = tk.Label(self, bg = "white",font= ("Verdana", 14), text = str(pid.getKp()))
        label2.place(x = 1265, y = 310+b, width = 80, height = 30)

        label3 = tk.Label(self, bg = "white",font= ("Verdana", 14), text = str(pid.getKi()))
        label3.place(x = 1265, y = 350+b, width = 80, height = 30)

        label4 = tk.Label(self, bg = "white",font= ("Verdana", 14), text = str(pid.getKd()))
        label4.place(x = 1265, y = 390+b, width = 80, height = 30)
        
class Teclado(tk.Frame):
                                    
    def __init__(self, parent, controller):
        global qsetpoint, btthome
        tk.Frame.__init__(self,parent)

        self.e = tk.Entry(self,  bg = "white",font= LARGE_FONT2)
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
                print ('Kp = ' + self.e.get())
                KpLabel.set(self.e.get())
                controller.show_frame(Visual)
                
            if K == 2:
                print ('Ki = ' + self.e.get())
                KiLabel.set(self.e.get())
                controller.show_frame(Visual)
                
            if K == 3:
                print('Kd = '+ self.e.get())
                KdLabel.set(self.e.get())
                controller.show_frame(Visual)
            if K == 4:
                controller.show_frame(Visual)
                consig = self.e.get()
                print('Consigna = '+consig)
                q.put('Manual')
                qsetpoint.put(float(consig))
                #textvarOpenFile.set('Consigna = {0} W'.format(consig))
            if K == 5:
                print('Ad = '+ self.e.get())
                Adlabel.set(self.e.get())
                controller.show_frame(Visual)   

        label = tk.Label(self, text = "Insert Constant", font = LARGE_FONT, background = 'snow', foreground = ColorTitulo)
        label.place(x = 350, y = 60, width = 800, height = 70) 

        button1 = tk.Button(self, text = "Confirm Value",  background = 'snow',font= MEDIUM_FONT,relief = 'solid', 
                              command = lambda: [K_selection(),self.e.delete(0,len(self.e.get()))], image = "")
        button1.place(x = 900, y =600, width = 300, height = 100)

        btthome = tk.Button(self, text = "Back", font= MEDIUM_FONT,borderwidth = 0, relief = 'solid', background='snow', 
                              command = lambda: controller.show_frame(Visual))
        btthome.place(x = 100, y = 65, width = 80, height = 80)
        
        button3 = tk.Button(self, text = "7", background = 'snow',font= MEDIUM_FONT,relief = 'solid', 
                              command = lambda: button_click(7))
        button3.place(x = 250, y = 280, width = 100, height = 100)
        
        button4 = tk.Button(self, text = "8",background = 'snow',font= MEDIUM_FONT,relief = 'solid', 
                              command = lambda: button_click(8))
        button4.place(x = 350, y = 280, width = 100, height = 100) 
        
        button4 = tk.Button(self, text = "9", background = 'snow',font= MEDIUM_FONT,relief = 'solid',  
                              command = lambda: button_click(9))
        button4.place(x = 450, y = 280, width = 100, height = 100)  
        
        button5 = tk.Button(self, text = "4", background = 'snow',font= MEDIUM_FONT,relief = 'solid', 
                              command = lambda: button_click(4))
        button5.place(x = 250, y = 380, width = 100, height = 100) 
        
        button6 = tk.Button(self, text = "5", background = 'snow',font= MEDIUM_FONT,relief = 'solid', 
                              command = lambda: button_click(5))
        button6.place(x = 350, y = 380, width = 100, height = 100)
        
        button7 = tk.Button(self, text = "6", background = 'snow',font= MEDIUM_FONT,relief = 'solid',  
                              command = lambda: button_click(6))
        button7.place(x = 450, y = 380, width = 100, height = 100)  
        
        button8 = tk.Button(self, text = "1", background = 'snow',font= MEDIUM_FONT,relief = 'solid', 
                              command = lambda: button_click(1))
        button8.place(x = 250, y = 480, width = 100, height = 100) 
        
        button9 = tk.Button(self, text = "2", background = 'snow',font= MEDIUM_FONT,relief = 'solid',
                              command = lambda: button_click(2))
        button9.place(x = 350, y = 480, width = 100, height = 100) 
        
        button10 = tk.Button(self, text = "3", background = 'snow',font= MEDIUM_FONT,relief = 'solid',
                              command = lambda: button_click(3))
        button10.place(x = 450, y = 480, width = 100, height = 100)
        
        button11 = tk.Button(self, text = ".", background = 'snow',font= MEDIUM_FONT,relief = 'solid',
                              command = lambda: button_click('.'))
        button11.place(x = 250, y = 580, width = 100, height = 100)
        
        button11 = tk.Button(self, text = "0", background = 'snow',font= MEDIUM_FONT,relief = 'solid', 
                              command = lambda: button_click(0))
        button11.place(x = 350, y = 580, width = 100, height = 100)  
        
        button11 = tk.Button(self, text = "Clear", background = 'snow',font= MEDIUM_FONT,relief = 'solid',
                              command = button_clear)
        button11.place(x = 450, y = 580, width = 100, height = 100) 

def configLabelParametros():
        global KpLabel, KiLabel, KdLabel,label2,label3,label4, Adlabel
        label1.config(textvariable=Adlabel)
        label2.config(textvariable=KpLabel)
        label3.config(textvariable=KiLabel)
        label4.config(textvariable=KdLabel)
def setLabelParametros():
        global KpLabel, KiLabel, KdLabel,label2,label3,label4, Adlabel
      #  Ad = qad.get()
      #  qad.put(Ad)
        kp = qkp.get()
        qkp.put(kp)
        ki = qki.get()
        qki.put(ki)
        kd = qkd.get()
        qkd.put(kd)
       # Adlabel.set(Ad)
        KpLabel.set(kp)
        KiLabel.set(ki)
        KdLabel.set(kd)

#--------------Create a DAC instance------------------
DAC = Adafruit_MCP4725.MCP4725(address=0x60, busnum=1)
#-----------Pila para el modo del emulador------------
q=queue.LifoQueue()
mode = 'Test'
q.put(mode)
qsetpoint = queue.LifoQueue()
qpower = queue.LifoQueue()
#-------------------------PID-------------------------
pid = PID(0.01,0.00005,0.004)
qkp=queue.LifoQueue()
qki=queue.LifoQueue()
qkd=queue.LifoQueue()
#qad=queue.LifoQueue()
qkp.put(0.01)
qki.put(0.00005)
qkd.put(0.004)
#--------------------Clase root=Tk()----------------
Interfaz = Emulador_UNIGRID()
Interfaz.configure(bg = 'snow')

#---------Añadir Imagen a botones---------------------
imag = Image.open("stopimg.png")
stopimg = ImageTk.PhotoImage(imag,master = Interfaz)
bttstop.configure(image=stopimg)
imag = Image.open("goimg.png")
goimg = ImageTk.PhotoImage(imag,master = Interfaz)
bttgo.configure(image=goimg)
imag = Image.open("homeimg.png")
homeimg = ImageTk.PhotoImage(imag,master = Interfaz)
btthome.configure(image=homeimg)
imag = Image.open('LogoU.png')
Uimg = ImageTk.PhotoImage(imag,master = Interfaz)
labelUni.configure(image=Uimg)
imag = Image.open('col.png')
cimg = ImageTk.PhotoImage(imag,master = Interfaz)
colimg.configure(image=cimg)

#---------Label de los parametros del PID-------------
Adlabel = StringVar(Interfaz)
KpLabel = StringVar(Interfaz)
KiLabel = StringVar(Interfaz)
KdLabel = StringVar(Interfaz)
Adlabel.set('1')
KpLabel.set(pid.getKp())
KiLabel.set(pid.getKi())
KdLabel.set(pid.getKd())
configLabelParametros()
setLabelParametros()
#-----------------------------------------------------
textvarOpenFile = StringVar(Interfaz)
#textvarOpenFile.set('')
#labelFileOk.configure(textvariable = textvarOpenFile)
#-----------------------------------------------------
iniciando = StringVar(Interfaz)
iniciando.set('')
labelini.configure(textvariable = iniciando)
#-----------------------------------------------------
actpid = StringVar(Interfaz)
actpid.set('')
#labelactpid.configure(textvariable = actpid)

npid = StringVar(Interfaz)
npid.set('')
#lpid.configure(textvariable = "")
#-----------------------------------------------------
actpow = StringVar(Interfaz)
actpow.set('')
labelpow.configure(textvariable = actpow)
#-----------------------------------------------------
Interfaz.attributes('-zoomed', True)
ani = animation.FuncAnimation(f, animate, interval = 100)
Interfaz.mainloop()
