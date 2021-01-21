import math
class spO2:
	def __init__(self, t, Rfilt, IRfilt):
		self.spO2     = 100
		self.spO2_avg = 100
		self.t        = t
		self.Rfilt    = Rfilt
		self.IRfilt   = IRfilt

	def spO2Calc(self):
		# print(len(self.t), len(self.Rfilt), len(self.IRfilt))
		
		Rfilt1 = self.Rfilt[:-1]
		Rfilt2 = self.Rfilt[1:]
		Rfilt3 = Rfilt2 - Rfilt1

		IRfilt1 = self.IRfilt[:-1]
		IRfilt2 = self.IRfilt[1:]
		IRfilt3 = IRfilt2 - IRfilt1

		r_red = abs(Rfilt3/Rfilt1)
		r_IR  = abs(IRfilt3/IRfilt1)

		r_red_threshold = sum(r_red)/len(r_red)
		r_IR_threshold = sum(r_IR)/len(r_IR)

		r_red_mean = 0
		red_count  = 0
		for i in range(len(r_red)):
			if r_red[i] >= r_red_threshold:
				r_red_mean += r_red[i]
				red_count += 1
			else:
				pass

		r_IR_mean = 0
		IR_count  = 0
		for i in range(len(r_IR)):
			if r_IR[i] >= r_IR_threshold:
				r_IR_mean += r_IR[i]
				IR_count += 1
			else:
				pass

		r_red_mean = r_red_mean / red_count
		r_IR_mean  = r_IR_mean  / IR_count

		R_value = r_red_mean/r_IR_mean
		
		if   0.10 <= R_value <= 0.35: self.spO2 = 100
		elif 0.35 <= R_value <= 0.40: self.spO2 = 99
		elif 0.40 <= R_value <= 0.45: self.spO2 = 98
		elif 0.45 <= R_value <= 0.47: self.spO2 = 97
		elif 0.47 <= R_value <= 0.49: self.spO2 = 96
		elif 0.49 <= R_value <= 0.51: self.spO2 = 95
		elif 0.51 <= R_value <= 0.53: self.spO2 = 94
		elif 0.53 <= R_value <= 0.55: self.spO2 = 93
		elif 0.55 <= R_value <= 0.57: self.spO2 = 92
		elif 0.57 <= R_value <= 0.59: self.spO2 = 91
		elif 0.59 <= R_value <= 0.61: self.spO2 = 90
		elif 0.61 <= R_value <= 0.64: self.spO2 = 89
		elif 0.64 <= R_value <= 0.67: self.spO2 = 88
		elif 0.67 <= R_value <= 0.70: self.spO2 = 87
		elif 0.70 <= R_value <= 0.73: self.spO2 = 86
		elif 0.73 <= R_value <= 0.76: self.spO2 = 85
		elif 0.76 <= R_value <= 0.79: self.spO2 = 84
		elif 0.79 <= R_value <= 0.82: self.spO2 = 83
		elif 0.82 <= R_value <= 0.85: self.spO2 = 82
		elif 0.85 <= R_value <= 0.88: self.spO2 = 81
		elif 0.88 <= R_value <= 0.91: self.spO2 = 80
		elif 0.91 <= R_value <= 0.94: self.spO2 = 79
		elif 0.94 <= R_value <= 0.97: self.spO2 = 78
		elif 0.97 <= R_value <= 1.00: self.spO2 = 77
		elif 1.00 <= R_value <= 1.03: self.spO2 = 76
		elif 1.03 <= R_value <= 1.06: self.spO2 = 75
		elif 1.06 <= R_value <= 1.09: self.spO2 = 74
		elif 1.09 <= R_value <= 1.12: self.spO2 = 73
		elif 1.12 <= R_value <= 1.15: self.spO2 = 72
		elif 1.15 <= R_value <= 1.18: self.spO2 = 71
		elif 1.18 <= R_value <= 1.21: self.spO2 = 70
		elif 1.21 <= R_value <= 1.24: self.spO2 = 69
		elif 1.24 <= R_value <= 1.27: self.spO2 = 68
		else:
			self.spO2 = 0

		self.spO2_avg = math.floor(0.5*(self.spO2 + self.spO2_avg))
		# print("spO2 = ", self.spO2, "   spO2 Avg = ", self.spO2_avg, "R_value =",R_value)
		print("spO2 = ", self.spO2, "    R_value =",R_value)
		return self.spO2