import numpy as np
from reduced_Map import reducedMap
import matplotlib
import matplotlib.cm as cm
import matplotlib.pyplot as plt

#Load Map File
MapFile = np.genfromtxt('3Colmap.csv', delimiter=',')
walls = np.array([x for x in MapFile if x[2]>0.001])
fig, ax = plt.subplots()
ax.scatter(walls[:,0], walls[:,1], label='Walls')


pos = [+3.3750e+00, +4.7500e+00]
rM = reducedMap(MapFile, pos[0], pos[1], 3)
walls1 = np.array([x for x in rM.cutMap if x[2]>0.001])
ax.scatter(walls1[:,0], walls1[:,1], label='First Cut Map')
ax.scatter(pos[0], pos[1], label='First Position')

pos = [-5, -5]
rM.propagateMotion(MapFile, pos[0], pos[1])
walls2 = np.array([x for x in rM.cutMap if x[2]>0.001])
ax.scatter(walls2[:,0], walls2[:,1], label='Second Cut Map')
ax.scatter(pos[0], pos[1], label='Second Position')

ax.legend(loc = 2)
plt.show()