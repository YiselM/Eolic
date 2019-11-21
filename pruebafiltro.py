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
import csv

def writecsv(filename,header,x,y,z):
    ifile = open(filename,mode='w')
    writer = csv.writer(ifile,delimiter = ' ')
    writer.writerow(header.split(' '))
    for i in range(len(x)):
	    xx = str(x[i]).replace('.',',')
	    yy = str(y[i]).replace('.',',')
	    zz = str(z[i]).replace('.',',')
	    writer.writerow([xx,yy,zz])
	    print(xx+" "+yy+" "+zz)
    ifile.close()

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
filteredVol = []
filteredCur = []
DataVoltage = []
DataCurrent = []
t = []
start = time.time()

#----------------------------------------FILTER SETUP----------------------------------------------
VolFilter = IIR2Filter(2,[5],'lowpass','butter',fs=1000)
CurFilter = IIR2Filter(2,[50],'lowpass','butter',fs=1000)

voltajedac = 5
voltbits=int((4096/5)*voltajedac)
DAC.set_voltage(voltbits)

time.sleep(3)
start = time.time()
for i in range(500):
    Current = ch0.voltage
    Voltage = ch3.voltage 
    DataVoltage.append(Voltage)
    DataCurrent.append(Current)
    filteredVol.append(VolFilter.filter(Voltage))
    filteredCur.append(CurFilter.filter(Current))
    timenow=(time.time()-start)
    t.append(timenow)
    print(str(i)+' '+str(Current))
writecsv('PruebaFiltroCorriente.csv','t Corriente FiltroCorriente',t,DataCurrent,filteredCur)
writecsv('PruebaFiltroVoltaje.csv','t Voltaje FiltroVoltaje',t,DataVoltage,filteredVol)

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
plt.title('Data')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.subplot(1,2,2)
plt.plot(t,filteredCur)
plt.title('Data')
plt.xlabel('Time (s)')
plt.ylabel('Current (I)')

plt.show()




