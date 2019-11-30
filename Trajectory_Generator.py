import numpy as np

class trajectoryGen:
	def __init__(self, xG, yG, r = 0.4, kdes = 0.04, bdes = 0.85, katt = 3, batt = 3, krep = 0.11, brep = 15, xdim = 20, ydim = 20):
		self.xG = xG
		self.yG = yG
		self.r = r
		self.kdes = kdes
		self.bdes = bdes
		self.katt = katt
		self.batt = batt
		self.krep = krep
		self.brep = brep
		self.xdim = xdim
		self.ydim = ydim

	def attGrad(self, xD, yD):
		disG = np.sqrt((xD - self.xG)**2 + (yD - self.yG)**2)
		if disG < self.batt:
			att = self.katt*np.array([xD - self.xG, yD - self.yG])
		else:
			att = self.katt*self.batt*np.array([xD - self.xG, yD - self.yG])/disG
		return att

	def repGrad(self, xD, yD, Map):
		xVals = np.unique(Map[:,0])
		yVals = np.unique(Map[:,1])
		s = 0*(max(xVals[1] - xVals[0], yVals[1] - yVals[0])) #Radius of obstacle
		#print('S = ', s)
		rep = 0
		negdis = False

		for i,p in enumerate(Map):
			if p[2] > 0.001:
				xO = p[0]
				yO = p[1]
				disO = np.sqrt((xO - xD)**2 + (yO - yD)**2) - s - self.r
				if disO < 0:
					# print('original distance = ', np.sqrt((xO - xD)**2 + (yO - yD)**2))
					# print('dis0 = ',disO)
					negdis = True
				gradDis = np.array([xD - xO, yD - yO])/np.sqrt((xD - xO)**2 + (yD - yO)**2)

				if disO < self.brep:
					repi = -self.krep*(1/disO - 1/self.brep)*((1/disO)**2)*gradDis
				else:
					repi = 0

				rep += repi

		return rep, negdis

	def genDesired(self, xD, yD, Map):
		att = self.attGrad(xD, yD)
		rep, negdis= self.repGrad(xD, yD, Map)
		grad = att + rep
		oldD = np.array([xD, yD])
		kGradnorm = np.linalg.norm(self.kdes*grad)

		if kGradnorm <= self.bdes:
			# print('Variation = ',- self.kdes*grad)
			newD = oldD  - self.kdes*grad
		else:
			# print('Variation = ',- self.bdes*grad*self.kdes/kGradnorm)
			newD = oldD - self.bdes*grad*self.kdes/kGradnorm

		# print('Att Grad = ',att)
		# print('Rep Grad = ',rep)
		# print('NewD = ',newD)
		print('Negative Distance = ',negdis)
		return newD[0], newD[1]
