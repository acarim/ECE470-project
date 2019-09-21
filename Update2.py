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

import numpy as np


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

import time

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)

    #Retrieve Handles
    errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'bubbleRob_leftMotor',vrep.simx_opmode_oneshot_wait)
    print('Left_err: ',errorCode)
    errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,'bubbleRob_rightMotor',vrep.simx_opmode_oneshot_wait)
    print('Right_err: ',errorCode)
    errorCode,sensor_handle=vrep.simxGetObjectHandle(clientID,'bubbleRob_sensingNose',vrep.simx_opmode_oneshot_wait)
    print('Sensor_err: ',errorCode)

    if res==vrep.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)
    time.sleep(2)

    #Initial Velocities
    v = 0.05*50;
    away = True

    # Now retrieve streaming data (i.e. in a non-blocking fashion):
    startTime=time.time()
    vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,vrep.simx_opmode_streaming) # Initialize streaming
    while time.time()-startTime < 10:

        errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,v, vrep.simx_opmode_streaming)
        errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,v, vrep.simx_opmode_streaming)
        #print('err: ',errorCode)

        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_handle,vrep.simx_opmode_streaming) 
        print('Sensor: ',np.linalg.norm(detectedPoint))

        if np.linalg.norm(detectedPoint) > 0 & away:
            v = -v
            away = False


    # Now send some data to V-REP in a non-blocking fashion:
    vrep.simxAddStatusbarMessage(clientID,'Finished execution.',vrep.simx_opmode_oneshot)

    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
