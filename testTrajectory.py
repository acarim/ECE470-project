import numpy as np
from reduced_Map import reducedMap
from Trajectory_Generator import trajectoryGen
import matplotlib
import matplotlib.cm as cm
import matplotlib.pyplot as plt

#Load Map File
MapFile = np.genfromtxt('3Colmap.csv', delimiter=',')
walls = np.array([x for x in MapFile if x[2]>0.001])
fig, ax = plt.subplots()
ax.scatter(walls[:,0], walls[:,1], label='Walls')


pos = [8.16, 3.1]
rM = reducedMap(MapFile, pos[0], pos[1], 3)
walls1 = np.array([x for x in rM.cutMap if x[2]>0.001])

goal = [-8.16, -3.1]
ax.scatter(goal[0], goal[1], label='Goal Position')

tG = trajectoryGen(goal[0], goal[1])
xD = pos[0]
yD = pos[1]
desired = np.array(pos)
actual = np.array(pos)

for i in range(100):
	xA = xD #+ float(np.random.rand(1)*3) - float(np.random.rand(1)*3)
	yA = yD #+ float(np.random.rand(1)*3) - float(np.random.rand(1)*3)
	rM.propagateMotion(MapFile, xA, yA)
	xD, yD = tG.genDesired(xA, yA, rM.cutMap)
	desired = np.vstack((desired, np.array([xD, yD])))
	actual = np.vstack((actual, np.array([xA, yA])))


ax.scatter(desired[:, 0], desired[:, 1], label = 'Desired Positions')
ax.scatter(actual[:, 0], actual[:, 1], label = 'Actual Positions')
ax.legend(loc = 2)
plt.show()