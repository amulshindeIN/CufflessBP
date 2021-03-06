# record data from comport using pyserial python module

import itertools
import time
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

##----------------------------------------------------------
# set initials parameters
i = 0
nsamples = 250
fileName = "pulse_"
fileName = fileName + time.strftime("%Y%m%d-%H%M%S") +'.txt'   
timeValueArray 	= np.zeros(nsamples)
redValueArray 	= np.zeros(nsamples)
irValueArray	= np.zeros(nsamples)

##----------------------------------------------------------
# read data from comport

serialPort	= 'COM3'
baudRate	= 9600
ser 		= serial.Serial(serialPort, baudRate, timeout=1)


while i < nsamples:

	arduinoData = ser.readline().decode('ascii')
	result = [k.strip() for k in arduinoData.split(',')]

	try:
		timeValueArray[i]  	= result[0]
		redValueArray[i]  	= result[1]
		irValueArray[i]   	= result[2]
		print(timeValueArray[i], redValueArray[i], irValueArray[i])
	except (ValueError, IndexError) :
		pass
	i+=1

'''
	timeValueArray[i]  	= result[0]
	redValueArray[i]  	= result[1]
	irValueArray[i]   	= result[2]
'''
	
	
try:
	np.savetxt(fileName, (timeValueArray, redValueArray, irValueArray))
	#np.savetxt('data_filtered.txt', (self.tfiltered, self.Rfiltered, self.IRfiltered))
except TypeError:
	print("TypeError occurred!!")
