import numpy as np

def updateVelocity(motor_velocity,dVel = 0.5):
    #Evolution of Velocity
    # if motor_velocity < 20*dVel:
    #     motor_velocity += dVel
    # else:
    #     motor_velocity -= dVel

    #realPos = (xA, yA)  Input variables
    #desPos = (xD, yD) Input variables
    """Kp = 1
    realPos = np.array((xA, yA))
    desPos = np.array((xD, yD))
    dist = numpy.linalg.norm(realPos-desPos) #Calculate absolute length between desPos and realPos
    motor_velocity = Kp*distVector*dVel"""

    motor_velocity = 5*dVel
    return motor_velocity

def updateSteering(left,steer_angle,realPos,desPos,heading,dSteer = 0.1,max_steer_angle = 0.5235987):
    # if left:
    #     steer_angle -= dSteer
    #     if steer_angle < -max_steer_angle:
    #         left = False
    # else:
    #     steer_angle += dSteer
    #     if steer_angle > max_steer_angle:
    #         left = True

    K = 0.5
    # realPos = np.array((xA, yA))
    # desPos = np.array((xD, yD))
    desiredVector = desPos[-1,:]-realPos[-1,:]
    x_axis_u = np.array([1, 0])
    print('realPos: ', realPos[-1,:])
    print('desPos: ', desPos[-1,:])
    print ('Desired vector: ', desiredVector)

    def unit_vector(vector):
        #Returns the unit vector of the vector.
        return vector / np.linalg.norm(vector)

    def angle_between(v1, v2):
        #Returns the angle in radians between vectors 'v1' and 'v2'
        v1_u = unit_vector(v1)
        v2_u = unit_vector(v2)

        A = np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)
        angle_between = np.arctan2(np.sqrt(1-(A)**2), A)
        return angle_between

    desiredAngle = angle_between(desiredVector, x_axis_u)       #In radians

    print('Heading angle: ', heading)
    print('Desired angle: ', desiredAngle)
    steer_angle = K*(desiredAngle-heading)

    if steer_angle<-max_steer_angle:
        steer_angle=-max_steer_angle
    if steer_angle>max_steer_angle:
        steer_angle=max_steer_angle

    print('Steer angle: ', steer_angle)

    return left, steer_angle


def Drive(motor_velocity, left, steer_angle, realPos, desPos, heading, dSteer = 0.1, max_steer_angle = 0.5235987, dVel = 0.5):
	left, steer_angle = updateSteering(left, steer_angle, realPos, desPos, heading, dSteer, max_steer_angle)
	motor_velocity = updateVelocity(motor_velocity, dVel)
	return motor_velocity, left, steer_angle
