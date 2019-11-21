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
i=0
start = time.time()
voltajedac = 4
voltbits=int((4096/5)*voltajedac)
DAC.set_voltage(voltbits)
print("set voltage to 4 time: "+str((time.time()-start)*1000)+"\n")
voltajedac = 0
voltbits=int((4096/5)*voltajedac)
DAC.set_voltage(voltbits)
print("set voltage to 0 time: "+str((time.time()-start)*1000)+"\n")
voltajedac = 3
voltbits=int((4096/5)*voltajedac)
DAC.set_voltage(voltbits)
print("set voltage to 3 time: "+str((time.time()-start)*1000)+"\n")
voltajedac = 0
voltbits=int((4096/5)*voltajedac)
DAC.set_voltage(voltbits)
print("set voltage to 0 time: "+str((time.time()-start)*1000)+"\n")
voltajedac = 5
voltbits=int((4096/5)*voltajedac)
DAC.set_voltage(voltbits)
print("set voltage to 5 time: "+str((time.time()-start)*1000)+"\n")
