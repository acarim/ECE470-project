import numpy as np
from csvmap import Map
from Particle_Filter import particleFilter
import matplotlib
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import time


#Load Map File
MapFile = np.genfromtxt('3Colmap.csv', delimiter=',')
walls = np.array([x for x in MapFile if x[2]>0.001])

fig, ax = plt.subplots()
ax.scatter(walls[:,0], walls[:,1], label='Walls')

u_t = [1, 0]
heading = np.pi/2
distances = np.array([0, 0, 2, 2]) #Front, Back, Right, Left

pf = particleFilter()

for i in range(0,15):
	time.sleep(5)
	estimatePosition, particles = pf.runParticleFilter(u_t, MapFile, heading, distances)
	ax.scatter(particles[:][0], particles[:][1], label='Particles')
	ax.scatter(estimatePosition[:][0], estimatePosition[:][1], label='Position')
	ax.legend()
	plt.show()