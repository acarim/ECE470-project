import numpy as np
import pandas as pd

xdim = 20
ydim = 20
n = 300
wallThickness = 0.6
corridorWidth = 6
cuboidSize = 0.6
wallHeight = 1
#Rectangles is an array containing the properties of each rectangle in the scene: [xcenter, ycenter, xdim, ydim]
rectangles = [[0,0,cuboidSize,cuboidSize]]

#Sinusoidal track boundary
nsin = 50
xsin = np.linspace(-xdim/2,xdim/2,nsin)
for x in xsin:
	rectangles.append([x, 5*np.sin(x*2*np.pi/xdim) + corridorWidth*0.5, wallThickness, wallThickness])
	rectangles.append([x, 5*np.sin(x*2*np.pi/xdim) - corridorWidth*0.5, wallThickness, wallThickness])



def boundCompare(x,a,b):
	isin = False
	lower = min(a,b)
	higher = max(a,b)
	if x>lower and x<higher:
		isin = True
	return isin


X = np.linspace(-xdim/2,xdim/2,n)
Y = np.linspace(-ydim/2,ydim/2,n)
csv = 0.001*np.ones([n, n])

for i in range(0,len(X)):
	for j in range(0,len(Y)):
		x = X[j]
		y = Y[i]

		#First off: Define bounding walls for the scene
		if not(boundCompare(x,-(0.5*xdim - wallThickness),(0.5*xdim - wallThickness))) or not(boundCompare(y,-(0.5*ydim - wallThickness),(0.5*ydim - wallThickness))): 
			csv[i][j] = wallHeight

		for k in rectangles:
			if boundCompare(x, k[0] - 0.5*k[2], k[0] + 0.5*k[2]) and boundCompare(y, k[1] - 0.5*k[3], k[1] + 0.5*k[3]):
				csv[i][j] = wallHeight

		# if boundCompare(x,-0.5*corridorWidth,0.5*corridorWidth): #Main Corridor
		# 	if y<-(0.5*ydim - wallThickness) or y>(0.5*ydim - wallThickness):
		# 		csv[i][j] = wallHeight
		# 	else:
		# 		csv[i][j] = 0
		# else:
		# 	csv[i][j] = wallHeight
		# if boundCompare(x,0,cuboidSize) and boundCompare(y,cuboidSize,2*cuboidSize): #Cuboid 1
		# 	csv[i][j] = wallHeight

		# if boundCompare(x,0,cuboidSize) and boundCompare(y,-cuboidSize,-2*cuboidSize):
		# 	csv[i][j] = wallHeight



df = pd.DataFrame(csv)
df.to_csv('CSVmap.csv', header = False, index = False)