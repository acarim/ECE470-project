import numpy as np
from reduced_Map import reducedMap

class trajectoryGen:
	def __init__(self, xG, yG, r = 1, kdes = 0.04, bdes = 0.85, katt = 2, batt = 3, krep = 0.11, brep = 15, xdim = 20, ydim = 20):
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
				# disO = np.sqrt((xO - xD)**2 + (yO - yD)**2) - s - self.r
				# if disO < 0:
				# 	# print('original distance = ', np.sqrt((xO - xD)**2 + (yO - yD)**2))
				# 	# print('dis0 = ',disO)
				# 	negdis = True
				# gradDis = np.array([xD - xO, yD - yO])/np.sqrt((xD - xO)**2 + (yD - yO)**2)

				# if disO < self.brep:
				# 	repi = -self.krep*(1/disO - 1/self.brep)*((1/disO)**2)*gradDis
				# else:
				# 	repi = 0

				dx = xD - xO
				dy = yD - yO
				dz = 0
				dNorm = np.sqrt(dx*dx+dy*dy+dz*dz)
				dNorm = dNorm - (self.r + s)

				tempVec_2x = xD - xO
				tempVec_2y = yD - yO
				tempVec_2z = 0
				tempNorm_2 = np.sqrt(tempVec_2x*tempVec_2x+tempVec_2y*tempVec_2y+tempVec_2z*tempVec_2z)
				del_dx = (1/tempNorm_2) * tempVec_2x
				del_dy = (1/tempNorm_2) * tempVec_2y
				del_dz = (1/tempNorm_2) * tempVec_2z


				if dNorm < self.brep:
					tempNorm_3 = ((self.krep*(dNorm-self.brep))/self.brep)*(1/dNorm)*(1/dNorm)*(1/dNorm)
					del_frepx = tempNorm_3*del_dx
					del_frepy = tempNorm_3*del_dy
					del_frepz = tempNorm_3*del_dz
				
				else:
					del_frepx = 0.0;
					del_frepy = 0.0;
					del_frepz = 0.0;

				repi = np.array([del_frepx, del_frepy])
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
		# print('Negative Distance = ',negdis)
		return newD[0], newD[1]

	def genTrajectory(self, start, goal, Map, n = 200, sensorLength = 5):
		rM = reducedMap(Map, start[0], start[1], sensorLength)
		xD = start[0]
		yD = start[1]
		desired = start

		for i in range(n):
			rM.propagateMotion(Map, xD, yD)
			xD, yD = self.genDesired(xD, yD, rM.cutMap)
			desired = np.vstack((desired, np.array([xD, yD])))

		return desired
