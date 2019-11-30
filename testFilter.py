import numpy as np
from Particle_Filter import particleFilter
from reduced_Map import reducedMap
import matplotlib
import matplotlib.pyplot as plt
import time
import pandas as pd
start_time = time.time()



#Load Map File
MapFile = np.genfromtxt('3Colmap.csv', delimiter=',')
walls = np.array([x for x in MapFile if x[2]>0])
print("Load Map File: %s seconds" % (time.time() - start_time))
start_time = time.time()

#Simulate Robot Readings
def readingMap(pos, Map, heading, sensorLength):
	sensorN = 8
	readings = sensorLength*np.ones(sensorN)
	xVals = np.unique(Map[:,0])
	yVals = np.unique(Map[:,1])
	for i in range(0, sensorN):
		theta = heading + i*(2*np.pi)/sensorN
		ray = np.linspace(0, sensorLength, 20)    
		for d in ray:
			#Projection of sensor ray along its heading
			far = pos + d*np.array([np.cos(theta), np.sin(theta)])
			#if far[0]<max(Map[:,0]) and far[0]>min(Map[:,0]) and far[1]<max(Map[:,1]) and far[1]>min(Map[:,1]):
				#What point of the map corresponds to this projected point?
				# higherX = [i for i,x in enumerate(Map[:,0] >= far[0]) if x]
				# thatX = [i for i,x in enumerate(Map[:,0]) if x == Map[higherX[0],0]]
				# higherY = [i for i, x in enumerate(Map[thatX,1] >= far[1]) if x] 
				# higherY = [x + thatX[0] for x in higherY]

				# higherX = np.searchsorted(Map[:,0], far[0])
				# thatX = np.argwhere(Map[:,0] == Map[higherX,0])
				# higherY = np.searchsorted(Map[thatX[:,0],1], far[1]) + higherX
			newCx = np.argmin(abs(xVals - far[0]))
			newCy = np.argmin(abs(yVals - far[1]))
			higherY = newCx*len(yVals) + newCy
				#Is there a wall at this projected point?
			if Map[higherY,2]>0.001:
				readings[i] = d
				break
	return readings


u_t = [0, 0]
pos = [-9.1, -2.3]#[+3.3750e+00, +4.7500e+00]
sensorLength = 5
rM = reducedMap(MapFile, pos[0], pos[1], sensorLength)
print('rM.cutMap Lenght Initialized: ',len(rM.cutMap))

rM.propagateMotion(MapFile, pos[0], pos[1])
print('rM.cutMap Lenght Propagated: ',len(rM.cutMap))

print("Create Reduced Map: %s seconds" % (time.time() - start_time))
start_time = time.time()

heading = 0
distances = readingMap(pos, rM.cutMap, heading, sensorLength) #Front, Left, Back, Right

print("Map Reading: %s seconds" % (time.time() - start_time))
start_time = time.time()

print('Imposed Position: ', pos)
print('Front, Left, Back, Right Sensors: ', distances)

pf = particleFilter(MapFile, sensorLength = 5)

fig, ax = plt.subplots()
ax.scatter(walls[:,0], walls[:,1], label='Walls')
ax.scatter(pf.particles[:,0], pf.particles[:,1], label = 'Initial Particles')

print("Initialize Filter: %s seconds" % (time.time() - start_time))
start_time = time.time()

for i in range(0,1):
	estimatePosition, particles = pf.runParticleFilter(u_t, MapFile, heading, distances)
	err = np.linalg.norm(np.array(pos) - estimatePosition)
	print('Estimated Position: ', estimatePosition)
	print('Estimation Error: ', err)

print("Run Filter: %s seconds" % (time.time() - start_time))
start_time = time.time()

ranking = np.argsort(pf.weights, 0)
ranking = ranking[::-1,0]

data = np.hstack((np.array(pos), distances, np.array([9999]), np.array([0, 0])))
dis = np.transpose(np.array([[np.linalg.norm(pos - p) for p in pf.particles[ranking,:]]]))
sens = np.transpose(np.array([[np.linalg.norm(distances - p) for p in pf.readings[ranking,:]]]))
#print(ranking.shape, pf.particles[ranking,:].shape, pf.readings[ranking,:].shape, pf.weights[ranking,:].shape, dis.shape, sens.shape)
partdata = np.hstack((pf.particles[ranking,:], pf.readings[ranking,:], pf.weights[ranking,:], dis, sens))
data = np.vstack((data, partdata))
df = pd.DataFrame(data, columns = ['PosX', 'PosY', 'Nsensor', 'NWsensor', 'Wsensor', 'SWsensor', 'Ssensor', 'SEsensor', 'Esensor', 'NEsensor', 'Weight', 'Physical Distance', 'Sensor Distance'])
df.to_excel('Particle Data.xlsx')

#ranking = ranking[:10]


ax.scatter(pf.particles[:,0], pf.particles[:,1], label = 'Resampled Particles')#, c = np.ravel(pf.weights), cmap='gray', label='Particles')
#ax.scatter(pf.particles[ranking,0], pf.particles[ranking,1], label='Top 10 Weighted Particles')
ax.scatter(estimatePosition[:][0], estimatePosition[:][1], label='Estimated Position')
ax.scatter(pos[0], pos[1], label='Actual Position')
ax.legend(loc = 2)
print("Plot: %s seconds" % (time.time() - start_time))
start_time = time.time()
plt.show()


