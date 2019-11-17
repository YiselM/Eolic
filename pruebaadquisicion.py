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

while True:
	voltajedac = int(input('Ingrese voltaje deseado del vdf '))
	voltbits=int((4096/5)*voltajedac)
	DAC.set_voltage(voltbits)
	Current = ch0.voltage
	Voltage = ch3.voltage 
	Current = Current*1.4089+0.1326
	Voltage = Voltage*9.5853-0.1082
	time.sleep(1)
	print('Un voltaje de {0} produce {1} A y {2}V'.format(voltajedac,Current,Voltage))
	Op = input('Desea continuar? (s/n)')
	if Op=='s':
		pass
	else:
		break
