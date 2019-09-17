import os
import time
import board
import busio
import numpy as np
import matplotlib.pyplot as plt
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from IIR2Filter import IIR2Filter

# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create the ADC object using the I2C bus
ads = ADS.ADS1115(i2c)

# Create single-ended input on channel 0
ch0 = AnalogIn(ads, ADS.P0)
ch3 = AnalogIn(ads, ADS.P3)

# Create differential input between channel 0 and 1
dif01 = AnalogIn(ads, ADS.P0, ADS.P1)
dif23 = AnalogIn(ads, ADS.P2, ADS.P3)

ads.gain = 2/3
ads.data_rate = 860

print("| {0:^5} | {1:^5} | {2:^5} | {3:^5} | {4:^5} |".format('t','V','I','FiltV','FiltI'))
print('-' * 17)

file = open("/media/pi/KELLY/ADCDATA/data_ADC.csv", "w+")
i=0
if os.stat("/media/pi/KELLY/ADCDATA/data_ADC.csv").st_size == 0:
    file.write("Time;Voltage;Current;FilteredVoltage;FilteredCurrent\n")
VolFilter = IIR2Filter(1,[1],'lowpass','butter',fs=1000)
CurFilter = IIR2Filter(1,[1],'lowpass','butter',fs=1000)

filteredVol = []
filteredCur = []

DataVoltage = []
DataCurrent = []

t = []
start = time.time()

for x in range(2000):
        
    Current = ch0.voltage
    Voltage = ch3.voltage

    DataVoltage.append(Voltage)
    DataCurrent.append(Current)
    
    filteredVol.append(VolFilter.filter(Voltage))
    filteredCur.append(CurFilter.filter(Current)) 
    
    timenow=(time.time()-start)
    t.append(timenow)
    
    print("| {0:^5.3f} | {1:^5.3f} | {2:^5.3f} | {3:^5.3f} | {4:^5.3f} |".format(timenow,Voltage,Current,filteredVol[i],filteredCur[i]))
    
    
    file.write(str(timenow)+';'+str(Voltage)+';'+str(Current)+';'+str(filteredVol[i])+';'+str(filteredCur[i])+"\n")
    file.flush()
    
    i=i+1
file.close()

#t=np.arange(0,2000)
plt.figure(0)

plt.subplot(1,2,1)
plt.plot(t,DataVoltage)
plt.title('Data')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.subplot(1,2,2)
plt.plot(t,DataCurrent)
plt.title('Data')
plt.xlabel('Time (s)')
plt.ylabel('Current (I)')

plt.figure(1)
plt.subplot(1,2,1)
plt.plot(t,filteredVol)
plt.title('Filtered Voltage')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.subplot(1,2,2)
plt.plot(t,filteredCur)
plt.title('Filtered Current')
plt.xlabel('Time (s)')
plt.ylabel('Current (I)')


plt.show()
