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


pos = [2.1, 3.4]
rM = reducedMap(MapFile, pos[0], pos[1], 3)
xVals = np.unique(rM.cutMap[:,0])
yVals = np.unique(rM.cutMap[:,1])
print(len(rM.cutMap))
print(rM.maxDim)
walls1 = np.array([x for x in rM.cutMap if x[2]>0.001])
ax.scatter(walls1[:,0], walls1[:,1], label='First Cut Map')
ax.scatter(pos[0], pos[1], label='First Position')

pos = [-9.1, -2.3]
rM.propagateMotion(MapFile, pos[0], pos[1])
print(len(rM.cutMap))
print(rM.maxDim)
walls2 = np.array([x for x in rM.cutMap if x[2]>0.001])
ax.scatter(walls2[:,0], walls2[:,1], label='Second Cut Map')
ax.scatter(pos[0], pos[1], label='Second Position')

pos = [0, 0]
rM.propagateMotion(MapFile, pos[0], pos[1])
print(len(rM.cutMap))
print(rM.maxDim)
walls3 = np.array([x for x in rM.cutMap if x[2]>0.001])
ax.scatter(walls3[:,0], walls3[:,1], label='Third Cut Map')
ax.scatter(pos[0], pos[1], label='Third Position')

ax.legend(loc = 2)
plt.show()