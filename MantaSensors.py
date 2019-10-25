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
from BasicDriving import Drive
from csvmap import Map
from Particle_Filter import particleFilter

#Load Map File
MapFile = np.genfromtxt('3Colmap.csv', delimiter=',')


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
    errorCode,frontSensor_handle=vrep.simxGetObjectHandle(clientID,'frontSensor',vrep.simx_opmode_oneshot_wait)
    print('frontSensor_handle: ',errorCode)
    errorCode,rearSensor_handle=vrep.simxGetObjectHandle(clientID,'rearSensor',vrep.simx_opmode_oneshot_wait)
    print('rearSensor_handle: ',errorCode)
    errorCode,rightSensor_handle=vrep.simxGetObjectHandle(clientID,'rightSensor',vrep.simx_opmode_oneshot_wait)
    print('rightSensor_handle: ',errorCode)
    errorCode,leftSensor_handle=vrep.simxGetObjectHandle(clientID,'leftSensor',vrep.simx_opmode_oneshot_wait)
    print('leftSensor_handle: ',errorCode)
    sensor_handles = [frontSensor_handle, rearSensor_handle, rightSensor_handle, leftSensor_handle]

    # topic = vrep.simxDefaultPublisher()

    #Maximum steer angle
    max_steer_angle=0.5235987
    #Maximum torque of the motor
    motor_torque=60
    
    #Control differentials
    dVel=0.5#1
    dSteer=0.1
    
    #Initial parameters
    steer_angle=0
    motor_velocity=dVel*10
    brake_force=100
    printCounter = 0;
    left = True
    sensorLength = 3
    sensorN = 4
    distances = sensorLength*np.ones(sensorN) #Front, Back, Right, Left

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
    while time.time()-startTime < 10:

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
            heading = euler[2] + 0.5*np.random.normal()

            #Scaled estimation of vehicle movement
            u_t = 0.5*motor_velocity*np.array([np.sin(heading), np.cos(heading)])

            for i in range(0,len(sensor_handles)):
                errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_handles[i],vrep.simx_opmode_streaming) 
                if detectionState:
                    distances[i] = np.linalg.norm(detectedPoint)
                
                else:
                   distances[i] = 100

            #Based on Vehicle Movement, Heading, and Sensor Distances, estimate position
            estimatePosition, particles = particleFilter().runParticleFilter(u_t, MapFile, heading, distances)

            print(estimatePosition)
            #print(distances)




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
else:
    print ('Failed connecting to remote API server')
print ('Program ended')



