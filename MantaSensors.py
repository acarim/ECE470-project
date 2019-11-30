# Make sure to have the server side running in V-REP: 
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
	import vrep
except:
	print ('--------------------------------------------------------------')
	print ('"vrep.py" could not be imported. This means very probably that')
	print ('either "vrep.py" or the remoteApi library could not be found.')
	print ('Make sure both are in the same folder as this file,')
	print ('or appropriately adjust the file "vrep.py"')
	print ('--------------------------------------------------------------')
	print ('')

import numpy as np
import time
from BasicDriving import Drive
from Particle_Filter import particleFilter
from reduced_Map import reducedMap
from Trajectory_Generator import trajectoryGen
import matplotlib
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import pandas as pd


print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
	print ('Connected to remote API server')

	# Now try to retrieve data in a blocking fashion (i.e. a service call):
	res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)

	#Retrieve Handles
	errorCode,vehicle_handle= vrep.simxGetObjectHandle(clientID,'Manta',vrep.simx_opmode_oneshot_wait)
	print('vehicle_handle: ',errorCode)
	errorCode,steer_handle= vrep.simxGetObjectHandle(clientID,'steer_joint',vrep.simx_opmode_oneshot_wait)
	print('steer_handle: ',errorCode)
	errorCode,motor_handle= vrep.simxGetObjectHandle(clientID,'motor_joint',vrep.simx_opmode_oneshot_wait)
	print('motor_handle: ',errorCode)
	errorCode,fl_brake_handle= vrep.simxGetObjectHandle(clientID,'fl_brake_joint',vrep.simx_opmode_oneshot_wait)
	print('fl_brake_handle: ',errorCode)
	errorCode,fr_brake_handle= vrep.simxGetObjectHandle(clientID,'fr_brake_joint',vrep.simx_opmode_oneshot_wait)
	print('fr_brake_handle: ',errorCode)
	errorCode,bl_brake_handle= vrep.simxGetObjectHandle(clientID,'bl_brake_joint',vrep.simx_opmode_oneshot_wait)
	print('bl_brake_handle: ',errorCode)
	errorCode,br_brake_handle= vrep.simxGetObjectHandle(clientID,'br_brake_joint',vrep.simx_opmode_oneshot_wait)
	print('br_brake_handle: ',errorCode)

	#Sensor Handles
	errorCode,Nsensor_handle=vrep.simxGetObjectHandle(clientID,'Nsensor',vrep.simx_opmode_oneshot_wait)
	print('Nsensor_handle: ',errorCode)
	errorCode,Ssensor_handle=vrep.simxGetObjectHandle(clientID,'Ssensor',vrep.simx_opmode_oneshot_wait)
	print('Ssensor_handle: ',errorCode)
	errorCode,Esensor_handle=vrep.simxGetObjectHandle(clientID,'Esensor',vrep.simx_opmode_oneshot_wait)
	print('Esensor_handle: ',errorCode)
	errorCode,Wsensor_handle=vrep.simxGetObjectHandle(clientID,'Wsensor',vrep.simx_opmode_oneshot_wait)
	print('Wsensor_handle: ',errorCode)
	errorCode,NEsensor_handle=vrep.simxGetObjectHandle(clientID,'NEsensor',vrep.simx_opmode_oneshot_wait)
	print('NEsensor_handle: ',errorCode)
	errorCode,SEsensor_handle=vrep.simxGetObjectHandle(clientID,'SEsensor',vrep.simx_opmode_oneshot_wait)
	print('SEsensor_handle: ',errorCode)
	errorCode,NWsensor_handle=vrep.simxGetObjectHandle(clientID,'NWsensor',vrep.simx_opmode_oneshot_wait)
	print('NWsensor_handle: ',errorCode)
	errorCode,SWsensor_handle=vrep.simxGetObjectHandle(clientID,'SWsensor',vrep.simx_opmode_oneshot_wait)
	print('SWsensor_handle: ',errorCode)
	sensor_handles = [Nsensor_handle, NWsensor_handle, Wsensor_handle, SWsensor_handle, Ssensor_handle, SEsensor_handle, Esensor_handle, NEsensor_handle]
	
	#Initial parameters: Driving
	goal = [-8.16, -3.1]
	dVel=0.5
	dSteer=0.1
	steer_angle=0
	motor_velocity=dVel*10
	brake_force=100
	left = True
	motor_torque=60
	max_steer_angle=0.5235987

	#Initial parameters: Sensing
	printCounter = 0
	begin = True
	sensorLength = 5
	sensorN = 8
	distances = sensorLength*np.ones(sensorN) #Front, Left, Back, Right

	#Load Map File
	MapFile = np.genfromtxt('3Colmap.csv', delimiter=',')
	walls = np.array([x for x in MapFile if x[2]>0])
	dimX = np.ptp(MapFile[:,0])
	dimY = np.ptp(MapFile[:,1])

	#Initialize Particle Filter
	pf = particleFilter(MapFile, dimX, dimY, sensorLength)

	#Initialize Trajectory Planner
	tG = trajectoryGen(goal[0], goal[1])

	if res==vrep.simx_return_ok:
		print ('Number of objects in the scene: ',len(objs))
	else:
		print ('Remote API function call returned with error code: ',res)

	#Release brakes if necessary
	vrep.simxSetJointForce(clientID,fr_brake_handle,0,vrep.simx_opmode_oneshot)
	vrep.simxSetJointForce(clientID,fl_brake_handle,0,vrep.simx_opmode_oneshot)
	vrep.simxSetJointForce(clientID,br_brake_handle,0,vrep.simx_opmode_oneshot)
	vrep.simxSetJointForce(clientID,bl_brake_handle,0,vrep.simx_opmode_oneshot)

	# Now retrieve streaming data (i.e. in a non-blocking fashion):
	startTime=time.time()
	vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,vrep.simx_opmode_streaming) # Initialize streaming
	while time.time()-startTime < 30:

		#Current Steer Position
		err, steer_pos = vrep.simxGetJointPosition(clientID,steer_handle,vrep.simx_opmode_streaming)

		#Extract linear angular velocity of rear wheels (Use in doubt)
		err, vlin, bl_wheel_velocity = vrep.simxGetObjectVelocity(clientID,bl_brake_handle,vrep.simx_opmode_oneshot_wait)
		err, vlin, br_wheel_velocity = vrep.simxGetObjectVelocity(clientID,br_brake_handle,vrep.simx_opmode_oneshot_wait)
		bl_wheel_velocity = np.linalg.norm(bl_wheel_velocity)
		br_wheel_velocity = np.linalg.norm(br_wheel_velocity)
		rear_wheel_velocity=(bl_wheel_velocity+br_wheel_velocity)/2
		linear_velocity=rear_wheel_velocity*0.09 

		vrep.simxSetJointForce(clientID,motor_handle,motor_torque,vrep.simx_opmode_oneshot)

		if printCounter % 10000 == 0:

			#Control of Vehicle
			motor_velocity, left, steer_angle = Drive(motor_velocity, left, steer_angle)
			vrep.simxSetJointTargetVelocity(clientID,motor_handle,motor_velocity,vrep.simx_opmode_oneshot)
			vrep.simxSetJointTargetPosition(clientID,steer_handle,steer_angle,vrep.simx_opmode_oneshot)

			#Simulated Compass Data
			isHead, euler = vrep.simxGetObjectOrientation(clientID, vehicle_handle, -1, vrep.simx_opmode_oneshot_wait)
			heading = (euler[2] + np.pi/2) + 0.5*np.random.normal()

			#True Vehicle Position
			isR, r = vrep.simxGetObjectPosition(clientID, vehicle_handle, -1, vrep.simx_opmode_oneshot_wait)
			pos = np.array(r[:2])

			#Vehicle Motion: To change once control allows the prediction of Ut
			if begin:
				u_t = np.array([0,0])
			else:
				u_t = pos - prev
			prev = pos

			#Read Sensor Data
			for i in range(0,len(sensor_handles)):
				errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_handles[i],vrep.simx_opmode_streaming) 
				if detectionState:
					distances[i] = np.linalg.norm(detectedPoint)
				else:
				   distances[i] = sensorLength

			#Initialize and propagate reduced map
			if begin:
				rM = reducedMap(MapFile, pos[0], pos[1], sensorLength)
				realPos = np.array(pos)
				estPos = np.array(pos)
				desPos = np.array(pos)
				t = np.array([])
				begin = False
			else:
				rM.propagateMotion(MapFile, estimatePosition[0], estimatePosition[1])

			#Based on Vehicle Movement, Heading, and Sensor Distances, estimate position
			# if printCounter % 100*10000 == 0:
			# 	pf = particleFilter(MapFile, dimX, dimY, sensorLength)
			estimatePosition, particles = pf.runParticleFilter(u_t, MapFile, heading, distances)

			#Evaluate error of estimated position
			err = np.linalg.norm(np.array(pos) - estimatePosition)
			print('Estimated Position at t = ',int(time.time()-startTime),' : ', estimatePosition, '. Estimation Error: ', err)

			#Based on estimated position, define desired trajectory
			xD, yD = tG.genDesired(estimatePosition[0], estimatePosition[1], rM.cutMap)

			#Store positions
			realPos = np.vstack((realPos, np.array(pos)))
			estPos = np.vstack((estPos, np.array(estimatePosition)))
			desPos = np.vstack((desPos, np.array([xD, yD])))
			t = np.append(t, time.time()-startTime)

	vrep.simxSetJointForce(clientID,motor_handle,0,vrep.simx_opmode_oneshot)
	vrep.simxSetJointForce(clientID,fr_brake_handle,brake_force,vrep.simx_opmode_oneshot)
	vrep.simxSetJointForce(clientID,fl_brake_handle,brake_force,vrep.simx_opmode_oneshot)
	vrep.simxSetJointForce(clientID,br_brake_handle,brake_force,vrep.simx_opmode_oneshot)
	vrep.simxSetJointForce(clientID,bl_brake_handle,brake_force,vrep.simx_opmode_oneshot)
	vrep.simxSetJointTargetVelocity(clientID,motor_handle,0,vrep.simx_opmode_oneshot)
	vrep.simxSetJointTargetPosition(clientID,steer_handle,0,vrep.simx_opmode_oneshot)

	# Now send some data to V-REP in a non-blocking fashion:
	vrep.simxAddStatusbarMessage(clientID,'Finished execution.',vrep.simx_opmode_oneshot)

	# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
	vrep.simxGetPingTime(clientID)

	# Now close the connection to V-REP:
	vrep.simxFinish(clientID)

	#Plot Evolution of Trajectories
	fig, ax = plt.subplots()
	ax.scatter(walls[:,0], walls[:,1], label='Walls')
	ax.scatter(estPos[1:,0], estPos[1:,1], label='Estimated Trajectory')#, c = np.ravel(t), cmap='winter')
	ax.scatter(realPos[1:,0], realPos[1:,1], label='Actual Trajectory')#, c = np.ravel(t), cmap='spring')
	ax.scatter(desPos[1:,0], desPos[1:,1], label='Desired Trajectory')#, c = np.ravel(t), cmap='summer')
	ax.legend(loc = 2)
	plt.show()

else:
	print ('Failed connecting to remote API server')
print ('Program ended')



