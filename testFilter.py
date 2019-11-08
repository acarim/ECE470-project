import numpy as np
from csvmap import Map
from Particle_Filter import particleFilter
import matplotlib
import matplotlib.cm as cm
import matplotlib.pyplot as plt
plt.close('all')
import time


#Load Map File
MapFile = np.genfromtxt('3Colmap.csv', delimiter=',')
walls = np.array([x for x in MapFile if x[2]>0.001])

#Simulate Robot Readings
def readingMap(pos, Map, heading):
	sensorLength = 3
	sensorN = 4
	readings = sensorLength*np.ones(sensorN)
	for i in range(0, sensorN):
		theta = heading + i*(2*np.pi)/sensorN
		ray = np.linspace(0, sensorLength, 20)    
		for d in ray:
			#Projection of sensor ray along its heading
			far = pos + d*np.array([np.cos(theta), np.sin(theta)])
			if far[0]<max(Map[:,0]) and far[0]>min(Map[:,0]) and far[1]<max(Map[:,1]) and far[1]>min(Map[:,1]):
				#What point of the map corresponds to this projected point?
				# higherX = [i for i,x in enumerate(Map[:,0] >= far[0]) if x]
				# thatX = [i for i,x in enumerate(Map[:,0]) if x == Map[higherX[0],0]]
				# higherY = [i for i, x in enumerate(Map[thatX,1] >= far[1]) if x] 
				# higherY = [x + thatX[0] for x in higherY]

				higherX = np.searchsorted(Map[:,0], far[0])
				thatX = np.argwhere(Map[:,0] == Map[higherX,0])
				higherY = np.searchsorted(Map[thatX[:,0],1], far[1]) + higherX

				#Is there a wall at this projected point?
				if Map[higherY,2]>0.001:
					readings[i] = d
					break
	return readings



u_t = [0, 0]
pos = [+3.3750e+00, +4.7500e+00]
heading = 0
distances = readingMap(pos, MapFile, heading) #Front, Left, Back, Right
print('Imposed Position: ', pos)
print('Front, Left, Back, Right Sensors: ', distances)

pf = particleFilter()

for i in range(0,1):
	estimatePosition, particles = pf.runParticleFilter(u_t, MapFile, heading, distances)
	err = np.linalg.norm(np.array(pos) - estimatePosition)
	print('Estimated Position: ', estimatePosition)
	print('Estimation Error: ', err)



fig, ax = plt.subplots()
ax.scatter(walls[:,0], walls[:,1], label='Walls')
ax.scatter(pf.particles[:,0], pf.particles[:,1], label='Particles')
ax.scatter(estimatePosition[:][0], estimatePosition[:][1], label='Estimated Position')
ax.scatter(pos[0], pos[1], label='Actual Position')
ax.legend(loc = 2)
plt.show()
