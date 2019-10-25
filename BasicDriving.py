

def updateVelocity(motor_velocity,dVel = 0.5):
    #Evolution of Velocity
    if motor_velocity < 20*dVel:
        motor_velocity += dVel
    else:
        motor_velocity -= dVel
    return motor_velocity

def updateSteering(left,steer_angle,dSteer = 0.1,max_steer_angle = 0.5235987):
    if left:
        steer_angle -= dSteer
        if steer_angle < -max_steer_angle:
            left = False
    else:
        steer_angle += dSteer
        if steer_angle > max_steer_angle:
            left = True
    return left, steer_angle


def Drive(motor_velocity, left, steer_angle, dSteer = 0.1, max_steer_angle = 0.5235987, dVel = 0.5):
	left, steer_angle = updateSteering(left, steer_angle, dSteer, max_steer_angle)
	motor_velocity = updateVelocity(motor_velocity, dVel)
	return motor_velocity, left, steer_angle