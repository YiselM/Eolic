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
DAC = Adafruit_MCP4725.MCP4725(address=0x60, busnum=1)

#Create Class PID


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

#----------------------------------------FILTER SETUP----------------------------------------------
VolFilter = IIR2Filter(2,[5],'lowpass','butter',fs=1000)
CurFilter = IIR2Filter(2,[200],'lowpass','butter',fs=1000)
PIDFilter = IIR2Filter(1,[20],'lowpass','butter',fs=1000)
#--------------------------------------------------------------------------------------------------
filteredVol = []
filteredCur = []

DataVoltage = []
DataCurrent = []
DataPower = []
DataOut = []

t = []
start = time.time()

#-----------------------------------------PID SETUP-----------------------------------------------
#pid = PID(0.55,0.9,0.005)
pid = PID(0.55,1,0.005)
pid.SetPoint=20
pid.setSampleTime(0.001)
feedback = 0
feedback_list = []
time_list = []

pidmin = 0 
pidmax = 5
# -----------------------------------------------------------------------------------------------

global voltajedac
voltajedac = 0
DAC.set_voltage(voltajedac)        


import threading
import time

flagcur = False
flagvol = False
curlist = []
vollist = []
powlist = []
start = time.time()

class myThread1 (threading.Thread):

    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter

    def run(self):
       # print("Starting " + self.name + "\n")
        run_emulator(self.name, 0.001, self.counter)
        #print("Exiting " + self.name + "\n")

class myThread2 (threading.Thread):

    def __init__(self, threadID, name, counter):

        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter

    def run(self):
        #print ("Starting " + self.name + "\n")
        run_emulator(self.name, 0.001, self.counter)        
        #print ("Exiting " + self.name + "\n")

class myThread3 (threading.Thread):

    def __init__(self, threadID, name, counter):

        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter

    def run(self):
        #print ("Starting " + self.name + "\n")
        run_emulator(self.name, 0.001, self.counter)        
        #print ("Exiting " + self.name + "\n")

def run_emulator(threadName, delay, counter):
    global flagcur,flagvol,voltajedac
    for i in range(2000):
    #while True:
        time.sleep(delay)
        if threadName=="Midiendo y Filtrando Corriente":
            print("CUR")
            try:
                Current = ch0.voltage
                #DataCurrent.append(CurFilter.filter(Current))
                DataCurrent.append(Current)
                DataCurrent[counter]=DataCurrent[counter]*1.4089+0.1326
                flagcur=True
            except IOError:
                print ("IO ERROR")
                counter-=1
                            
            #print("%s: %s %s" % (threadName, time.ctime(time.time()), counter)+"\n")
            #print("Corriente: "+str(DataCurrent[counter])+"\n")
            
        elif threadName=="Midiendo y Filtrando Voltaje":
            #print("VOL")
            try:
                Voltage = ch3.voltage
                #DataVoltage.append(VolFilter.filter(Voltage))
                DataVoltage.append(Voltage)
                DataVoltage[counter]=DataVoltage[counter]*9.5853-0.1082
                flagvol=True
            except IOError:
                print ("IO ERROR")
                counter-=1
                       
            #print("%s: %s %s" % (threadName, time.ctime(time.time()), counter)+"\n")
            #print("Voltaje: "+str(DataVoltage[counter])+"\n")
            
        elif threadName=="Realizando PID":
            
            if(flagcur==True and flagvol==True):
                DataPower.append(DataVoltage[counter]*DataCurrent[counter])
                timenow=(time.time()-start)
                t.append(timenow)
                if (timenow > 0 and timenow < 15):
                    pid.SetPoint=20
                elif (timenow > 15 and timenow < 30):
                    pid.SetPoint=30
                elif (timenow > 30 ):
                    pid.SetPoint=10
                # --------------------------------------- PID CONTROLLER------------------------------------------
                pid.update(DataPower[counter])
                output = pid.output

                if pid.SetPoint > 0:
                    voltajedac = voltajedac + (output - (1/(counter+1)))
                if voltajedac < pidmin:
                    voltajedac = pidmin
                elif voltajedac > pidmax:
                    voltajedac = pidmax
                # ---------------------------------------------DAC------------------------------------------------

                DataOut.append(PIDFilter.filter(voltajedac))
                voltbits=int((4096/5)*voltajedac)
                DAC.set_voltage(voltbits)    
    
                # ------------------------------------------------------------------------------------------------
                #print("%s: %s %s" % (threadName, time.ctime(time.time()), counter)+"\n")
                #print(str(counter)+" potencia: "+str(powlist[counter])+"tiempo: "+str(time.time()-start)+"\n")
                print(str(counter)+" Potencia: "+str(DataPower[counter])+" "+str((time.time()-start)*1000)+"\n")
                flagcur=False
                flagvol=False
            else:
                #print("ELSE")
                counter-=1
                
        counter+=1

        
        
threadLock = threading.Lock()

# Create new threads

thread1 = myThread1(1, "Midiendo y Filtrando Corriente", 0)
thread2 = myThread2(2,"Midiendo y Filtrando Voltaje", 0)
thread3 = myThread1(3,"Realizando PID", 0)


# # Start new threads

# thread3.start()

# threadLock.acquire()
# thread1.start()
# thread2.start()
# thread1.join()
# thread2.join()
# threadLock.release()

# thread3.join()

thread1.start()
thread2.start()
thread3.start()
thread1.join()
thread2.join()
thread3.join()

# plt.subplot(1,2,1)
# plt.plot(t,DataVoltage)
# plt.title('Data')
# plt.xlabel('Time (s)')
# plt.ylabel('Voltage (V)')
# plt.ylim(0,60)
# plt.subplot(1,2,2)
# plt.plot(t,DataCurrent)
# plt.title('Data')
# plt.xlabel('Time (s)')
# plt.ylabel('Current (I)')
# plt.ylim(0,60)

# plt.figure(1)
# plt.plot(t,DataPower)
# plt.title('Data')
# plt.xlabel('Time (s)')
# plt.ylabel('Power (W)')
# plt.ylim(0,60)

print ("Exiting Main Thread")

