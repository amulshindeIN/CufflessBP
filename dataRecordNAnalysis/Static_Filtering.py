import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.fftpack import fft
import scipy as sc
import scipy.signal as sps
from scipy.signal import butter, lfilter, freqz, filtfilt, savgol_filter
from scipy.signal import find_peaks, peak_widths
import math
from spO2 import *


# original record
#path = 'C:\\Users\\amul\\Documents\\Pulse_Oximeter\\Data\\190919\\'
file = 'pulse_20210120-190721'
ext  = '.txt'

t, RED, IR = np.loadtxt(file + ext)

#     				x limits				#

# print("IR_MAX =   ", IR.max())
# print("IR_min =   ", IR.min())
# print("time_min = ", t.min())
# sys.exit()


# Frequency Axis limits
fxAxis = [0.1, 5]
fyAxis = [-0.5, 500]

# Low pass filter Data Axis limits
x1 = 2654
x2 = x1 + 250
y1 = 29500
y2 = y1 + 600

xAxis = [t.min(), t.max()]
yAxis = [y1, y2]

# After cascade of filters
# Axis limits
y3 = 300
y3axis = [-y3, y3] 

#############################################
#     				fft 					#
#############################################

x = t
N = len(x)
NFFT = N

ts = (x[N-1] - x[0])/(N-1) 	# reconstructing time step
fs = 1/ts
samplingFreq = fs/2

f = samplingFreq * np.arange(-1,1, 2/NFFT)	# frequency axis
f = samplingFreq * np.arange(2/NFFT,1, 2/NFFT)	# frequency axis

IRf 	= fft(IR)
IRfAbs 	= np.absolute(IRf)/NFFT
IRfAbs 	= IRfAbs[0:int(np.floor((NFFT-1)/2))]     # Only half the frequency axis is considered for plotting

REDf 	= fft(RED)
REDfAbs = np.absolute(REDf)/NFFT
REDfAbs = REDfAbs[0:int(np.floor((NFFT-1)/2))]


#############################################
#			Filtered signal  				#
#############################################

# ################### Low pass @ 6Hz ##################################

cutoffFreq = np.array(9, dtype= float)		# in Hz
Wn = cutoffFreq/samplingFreq
btype = 'low'	# 'band'/'high'/'low'
b, a = sps.butter(6, Wn, btype)
# w, h = sps.freqz()
IRfiltered = filtfilt(b, a, IR)
REDfiltered = filtfilt(b, a, RED)

################### High pass @ 1Hz ##################################
cutoffFreq = np.array(1, dtype= float)		# in Hz
Wn = cutoffFreq/samplingFreq
btype = 'high'	# 'band'/'high'/'low'
b, a = sps.butter(6, Wn, btype)
IRfiltered_highpass = filtfilt(b, a, IRfiltered)
REDfiltered_highpass = filtfilt(b, a, REDfiltered)

################### Low pass @ 4Hz ##################################

cutoffFreq = np.array(4, dtype= float)		# in Hz
Wn = cutoffFreq/samplingFreq
btype = 'low'	# 'band'/'high'/'low'
b, a = sps.butter(6, Wn, btype)
IRfiltered = filtfilt(b, a, IR)
IRfiltered_highpass_bandpass = filtfilt(b, a, IRfiltered_highpass)
REDfiltered_highpass_bandpass = filtfilt(b, a, REDfiltered_highpass)
#######################################################################
# print(len(IRfiltered_highpass_bandpass), len(REDfiltered_highpass_bandpass))



########## BPM Calculation & Window detection #############
RED_BPM = -REDfiltered_highpass_bandpass + 1000
IR_BPM  = -IRfiltered_highpass_bandpass  + 1000
peaks, _ = find_peaks(RED_BPM, height=1001)
peaks1, _ = find_peaks(IR_BPM, height=1002)

timeDiff = t[peaks1]
# print(t[peaks])
timeDiff = timeDiff[1:] - timeDiff[:-1]
try:
	timeDiff = sum(timeDiff)/len(timeDiff)
	BPM      = math.floor(60/timeDiff)
	print("Heart Rate = ", BPM)
except:
	print("***************Please check BPM Calculation section")
############################################################


spO2 = spO2(t, REDfiltered_highpass_bandpass + 1000, IRfiltered_highpass_bandpass+1000)
spO2 = spO2.spO2Calc()


#########################################################################
#							Figures										#
#########################################################################

fig , (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1,figsize=(10, 5))
# string =  "IR:0F - RED:1F" + " - BPM:" + str(BPM) + " - spO2:" + str(spO2) + "%"
# fig.suptitle(string, x=0.5, y=0.95)

####    Raw plot with LOW pass fitting ###############################

ax1.plot(t, IR, 'k', label="IR", linewidth=1, alpha=1)
# ax1.plot(x, IRfiltered, 'k-', label = '', linewidth=1)
ax1.set_xlabel("Time (second)")
ax1.set_xlim(xAxis)
# ax1.set_ylim(yAxis)
ax1.legend()

color = 'tab:red'
ax4 = ax1.twinx()
ax4.plot(t, RED, color=color, label="RED", linewidth=1, alpha=1)
# ax4.plot(x, REDfiltered, color=color, label = '', linewidth=1)
ax4.tick_params(axis='y', labelcolor=color)
ax4.legend()

##### Frequency plot  #################################################
# f = f[:-1]
ax2.plot(f, IRfAbs, 'k-' , label = 'IR',linewidth = 1.5)
ax2.plot(f, REDfAbs, 'r-' , label = 'RED',linewidth = 1.5)
ax2.set_xlabel("Frequency (Hz)")
ax2.set_ylim(fyAxis)
ax2.set_xlim(fxAxis)
# ax2.set_legend()
ax2.set_xticks(np.arange(0,5.5,step=0.5))
# ax2.tick_params(axis='x')
######################################################################

#############  Cascade of Low pass and High pass #######################

# ax3.plot(x, IRfiltered_highpass, 'k-', label = 'Low & High pass', linewidth=2, alpha=0.25)
ax3.plot(x, IRfiltered_highpass_bandpass, 'k-', label = 'Low & High pass', linewidth=2)
ax3.set_xlabel("Time (second)")
ax3.set_xlim(xAxis)
ax3.set_ylim(y3axis)

color = 'tab:red'
ax5 = ax3.twinx()
# ax5.plot(x, REDfiltered_highpass, 'r-', label = '', linewidth=2, alpha=0.25)
ax5.plot(x, REDfiltered_highpass_bandpass, 'r-', label = 'Low & High pass', linewidth=2)
ax5.set_ylim(y3axis)
ax5.tick_params(axis='y', labelcolor=color)

plt.tight_layout()
plt.savefig(file+'.png')
plt.show()


## Signal to Noise Calculation

IR_SGolay  = savgol_filter(IR, 15, 2)
RED_SGolay = savgol_filter(RED, 5, 2)

IR_Noise   = abs(IR  - IR_SGolay)
RED_Noise  = abs(RED - RED_SGolay)


np.set_printoptions(precision=2)  # For compact display.
SnrIR  = 10*np.log10(np.square(IR_SGolay)/np.square(IR_Noise))
SnrRED = 10*np.log10(np.square(RED_SGolay)/np.square(RED_Noise))

print('SnrIR  = ', int(np.mean(SnrIR)))
print('SnrRED = ', int(np.mean(SnrRED)))

plt.plot(t, SnrIR, "k")
plt.plot(t, SnrRED, "r")
# plt.show()