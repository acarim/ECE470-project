import numpy as np

#In order to reduce the computational cost of the particle filter and the trajectory generator, instead of reading every time through the whole
#MapFile as is extracted from its .csv file, it is more convenient to read only a particular region of interest. In this way, this script intends
# to extract, for a specified position in the map, only the points which are considered of interest. This region of interest is defined as the region
# within a 1.5*sensorLength radius of the vehicle.

class reducedMap:

	def __init__(self, Map, x0, y0, sensorLength = 3):

		self.xValues = np.unique(Map[:,0])
		self.yValues = np.unique(Map[:,1])

		newCx = np.argmin(abs(self.xValues - x0))
		newCy = np.argmin(abs(self.yValues - y0))
		self.xCenter = self.xValues[newCx]
		self.yCenter = self.yValues[newCy]
		self.iCenter = newCx
		self.jCenter = newCy

		#Cut the Map in X
		xrange = [max(self.xCenter - 1.5*sensorLength, -np.ptp(self.xValues)/2), min(self.xCenter + 1.5*sensorLength, np.ptp(self.xValues)/2)]
		isinX = []
		for i,x in enumerate(self.xValues):
			if x >= xrange[0] and x <= xrange[1]:
				tempX = [len(self.yValues)*i + j for j in range(len(self.yValues))]
				isinX.extend(tempX)

		#Cut the Map in Y
		yrange = [max(self.yCenter - 1.5*sensorLength, -np.ptp(self.yValues)/2), min(self.yCenter + 1.5*sensorLength, np.ptp(self.yValues)/2)]
		isinY = []
		for i,y in enumerate(self.yValues):
			if y >= yrange[0] and y <= yrange[1]:
				tempY = [i + j*len(self.yValues) for j in range(len(self.xValues))]
				isinY.extend(tempY)

				self.isin = np.intersect1d(isinX, isinY)
				self.cutMap = Map[self.isin, :]

	def propagateMotion(self, Map, x1, y1):

		newCx = np.argmin(abs(self.xValues - x1))
		newCy = np.argmin(abs(self.yValues - y1))
		self.xCenter = self.xValues[newCx]
		self.yCenter = self.yValues[newCy]

		deltaX = newCx - self.iCenter
		deltaY = newCy - self.jCenter
		self.iCenter = newCx
		self.jCenter = newCy

		#Translation in X
		self.isin = self.isin + deltaX*len(self.yValues)

		#Translation in Y
		self.isin = self.isin + deltaY

		#Check for Points outside the map
		for i,j in enumerate(self.isin):
			if j > len(self.yValues)*len(self.xValues):
				self.isin[i] = len(self.yValues)*len(self.xValues)
			elif j < 0:
				self.isin[i] = 0
		self.isin = np.unique(self.isin)
		self.cutMap = Map[self.isin, :]




