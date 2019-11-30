import numpy as np
from reduced_Map import reducedMap

class particleFilter:
	
	def __init__(self, x_dim = 20, y_dim = 20, sensorLength = 3):
		
		self.numParticles = 500
		self.dimension = 2
		self.std = 5                                     
		self.curMax = [x_dim/2, y_dim/2]                     
		self.curMin = [-x_dim/2, -y_dim/2]                   
		self.resNoise = [x*0.01 for x in self.curMax]
		self.sensorLength = sensorLength
		self.sensorN = 8
		
		############# The initial particles are uniformely distributed #############
		
		# Generate uniformly distributed variables in x and y direction within [0, 1]
		# Spread these generated particles on the maze
		
		# Generate weight, initially all the weights for particle should be equal, namely 1/num_of_particles
		# weights should be something like [1/num_of_particles, 1/num_of_particles, 1/num_of_particles, ...]

		
		self.particles = np.random.uniform(-1,1,(self.numParticles,2))
		self.particles = np.multiply(self.particles,self.curMax)
		self.weights = (1/self.numParticles)*np.ones(self.numParticles)
		self.readings = np.zeros((self.numParticles, self.sensorN))
		# print('########## INITIAL PARTICLE DISTRIBUTION ##########', type(self.particles))
		# print(self.particles)

		
	def Sample_Motion_Model(self, u_t=0):
		
		########## Sample the Motion Model to Propagate the Particles ###########
		
		# For each particle in self.particles [[x1,y1], [x2,y2], ...], get the nextEstimate
		
		for i in range(0,self.numParticles):
			current = self.particles[i]
			propagated = current + u_t
			self.particles[i] = propagated
		# print('########## PROPAGATED PARTICLE DISTRIBUTION ##########', type(self.particles))
		# print(self.particles)
	
	def Measurement_Model(self, Map, heading, distances):
		
		##################### Measurement Motion Model #####################
		# Calculate distance between robot's postion and each particle's position
		# Calculate weight for each particle, w_t = exp(-distance**2/(2*self.std))
		
		# Collect all the particles' weights in a list
		# For all the weights of particles, normalized them
		# Update self.weights
		#Important: distances should follow a c-clockwise rotation from the vehicle's nose
		
		
		robotReading = distances
		w_t = np.zeros((self.numParticles,1))

		for i in range(0,self.numParticles):
			rM = reducedMap(Map, self.particles[i, 0], self.particles[i, 1])
			current = np.array(self.particles[i])
			particleReading = np.array(self.readingMap(current, rM.cutMap, heading))
			w_t[i] = np.exp( - np.linalg.norm(particleReading - robotReading)**2 / (2*self.std))
			self.readings[i, :] = particleReading
			if sum(particleReading) == 0: #Directly exclude particles in walls
				w_t[i] = 0


		k = sum(w_t)
		if k == 0:
			print('Sum of weights is null')
			self.weights = (1/self.numParticles)*np.ones(self.numParticles)
		else:
			self.weights = w_t/k

	
	def readingMap(self, pos, Map, heading):
		
		readings = self.sensorLength*np.ones(self.sensorN)
		xrange = [min(Map[:,0]), max(Map[:,0])]
		xVals = np.unique(Map[:,0])
		yVals = np.unique(Map[:,1])
		for i in range(0, self.sensorN):
			theta = heading + i*(2*np.pi)/self.sensorN
			ray = np.linspace(0, self.sensorLength, 20)    
			for d in ray:
				#Projection of sensor ray along its heading
				far = pos + d*np.array([np.cos(theta), np.sin(theta)])
				#if far[0] < xrange[1] and far[0] > xrange[0] and far[1]<max(Map[:,1]) and far[1]>min(Map[:,1]):
					#What point of the map corresponds to this projected point?
					# higherX = [i for i,x in enumerate(Map[:,0] >= far[0]) if x]
					# thatX = [i for i,x in enumerate(Map[:,0]) if x == Map[higherX[0],0]]
					# higherY = [i for i, x in enumerate(Map[thatX,1] >= far[1]) if x] 
					# higherY = [x + thatX[0] for x in higherY]

				newCx = np.argmin(abs(xVals - far[0]))
				newCy = np.argmin(abs(yVals - far[1]))
				higherY = newCx*len(yVals) + newCy

				if higherY>len(Map):
					print(far)
					print(newCx,newCy)
					print(higherY)
					print(len(yVals)*len(xVals))
					# higherX = np.searchsorted(Map[:,0], far[0])
					# thatX = np.argwhere(Map[:,0] == Map[higherX,0])
					# higherY = np.searchsorted(Map[thatX[:,0],1], far[1]) + higherX

					#Is there a wall at this projected point?
				if Map[higherY,2]>0.001:
					readings[i] = d
					break
		return readings
	
	def calcPosition(self):
		
		############# Calculate the position update estimate ###############
		## TODO: return a list with two elements [x,y],  estimatePosition
		
		# For all the particles in direction x and y, get one estimated x, and one estimated y
		# Hint: use the normalized weights, self.weights, estimated x, y can not be out of the
		# boundary, use self.curMin, self.curMax to check
		
		## Your Code start from here
		ranking = np.argsort(self.weights, axis = 0)
		ranking = ranking[::-1]
		ranking = ranking[:int(np.floor(self.numParticles/20))]

		weighted = sum(np.multiply(self.particles[ranking, :],self.weights[ranking]))
		weighted = weighted[0,:]
		x = max(min(weighted[0], self.curMax[0]), self.curMin[0])
		y = max(min(weighted[1], self.curMax[1]), self.curMin[1])
		estimatePosition = [x, y]

		return estimatePosition
	
	# Method 1
	# def resampling(self):
		
	# 	## TODO: Comment the code below:
	# 	"""
		
		
	# 	"""
	# 	newParticles = []
	# 	N = len(self.particles)
	# 	cumulative_sum = np.cumsum(self.weights)
	# 	cumulative_sum[-1] = 1
	# 	for i in range(N):
	# 		randomProb = np.random.uniform()
	# 		index = np.searchsorted(cumulative_sum, randomProb)
	# 		newParticles.append(self.particles[index])
	# 	self.particles = newParticles
		

		
	# ## Method 2: Roulette Wheel
	def resampling(self):
		
	    ## TODO: Comment the code below:
	    """
		
	    """
	    newParticles = []
	    N = len(self.particles)
	    index = int(np.random.random() * N)
	    beta = 0
	    mw = np.max(self.weights)
	    for i in range(N):
	        beta += np.random.random() * 2.0 * mw
	        while beta > self.weights[index]:
	            beta -= self.weights[index]
	            index = (index + 1) % N
	        current = self.particles[index]
	        newParticles.append(np.ndarray.tolist(current))
	    self.particles = np.asarray(newParticles)
	    # print('########## RESAMPLED PARTICLE DISTRIBUTION ##########', type(self.particles))
	    # print(self.particles)
	
	def runParticleFilter(self, u_t, Map, heading, distances):
		

		self.Sample_Motion_Model(u_t)
		self.Measurement_Model(Map, heading, distances)
		estimatePosition = self.calcPosition()
		self.resampling()
		particles = self.particles
		return estimatePosition, particles

