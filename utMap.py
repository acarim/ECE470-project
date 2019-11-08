import numpy as np

##################### SCRIPT DESCRIPTION #####################
# Here, function utFromMap determines what the necessary u_t to propagate particle motion is, based on values for the steer position, 
# the motor velocity, and the heading.

def ut_fromMap(V, theta, heading, data, Vrange, thRange):
	whichV = np.searchsorted(Vrange, V)
	whichTh = np.searchsorted(thRange, theta)

	#A mean between both neighboring velocities is performed
	wVhigher = (V - Vrange[whichV - 1])/(Vrange[whichV] - Vrange[whichV - 1])
	wVlower = (Vrange[whichV] - V)/(Vrange[whichV] - Vrange[whichV - 1])

	#A mean between both neighboring steering positions is performed
	wthHigher = (theta - thRange[whichTh - 1])/(thRange[whichTh] - thRange[whichTh - 1])
	wthLower = (thRange[whichTh] - theta)/(thRange[whichTh] - thRange[whichTh - 1])

	vec = data[:, whichV, :]*wVhigher + data[:, whichV - 1, :]*wVlower
	print(vec.shape)
	vals = vec[whichTh, :]*wthHigher + vec[whichTh - 1, :]*wthLower
	print(vals.shape)
	ut_unrotated = vals[0:2]
	headingDiff = heading - vals[2]

	utx = np.cos(headingDiff)*ut_unrotated[0] - np.sin(headingDiff)*ut_unrotated[1]
	uty = np.sin(headingDiff)*ut_unrotated[0] + np.cos(headingDiff)*ut_unrotated[1]
	ut = np.array([utx, uty])
	return ut

npzfile = np.load('u_t.npz')
data = npzfile['data']
Vrange = npzfile['V']
thRange = npzfile['theta']
V = 3
theta = 0
heading = 0
print(thRange)
ut = ut_fromMap(V, theta, heading, data, Vrange, thRange)
print('With V = '+ str(V) + ', theta = ' + str(theta) + ' and heading = ' + str(heading))
print('u_t = ', ut)
