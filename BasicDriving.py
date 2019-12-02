import numpy as np

class Driver:

    def __init__(self, dSteer = 0.1, max_steer_angle = 0.5235987, dVel = 0.5):
        self.dSteer = dSteer
        self.max_steer_angle = max_steer_angle
        self.dVel = dVel
        self.index = 0

    def updateVelocity(self, dVel = 0.5):

        self.motor_velocity = 4*dVel

    def nextDesPos(self, realPos, desPosarr):
        # desPosarr = np.array([[-1.5,-2],[0,0],[1.5,2],[3,4]])

        desCut = desPosarr[self.index:,:]
        distVec = np.zeros(len(desCut))
        for i,p in enumerate(desCut):
            dist = np.linalg.norm(realPos[-1,:]-desCut[i,:]) #Check which point is the closest
            distVec[i] = dist
        i = np.argmin(distVec)
        if i > self.index :
            self.index = i

        if np.min(distVec) < 1:
            desPos = desCut[i+1]
            self.index = self.index + 1
        else:
            desPos = desCut[i]

        self.desPos = desPos
        return desPos

    def updateSteering(self, realPos, heading):

        K = 0.35

        desiredVector = self.desPos-realPos[-1,:]
        x_axis_u = np.array([1, 0])
        print('realPos: ', realPos[-1,:])
        print('desPos: ', self.desPos, 'Index = ', self.index)
        print ('Desired vector: ', desiredVector)

        def unit_vector(vector):
            #Returns the unit vector of the vector.
            return vector / np.linalg.norm(vector)

        def angle_between(v1, v2):
            cosang = np.dot(v1, v2)
            sinang = np.linalg.norm(np.cross(v1, v2))
            angle_between = np.arctan2(sinang,cosang)
            return angle_between

        desiredAngle = angle_between(desiredVector, x_axis_u)       #In radians
        if desiredVector[0]<0:
            if desiredVector[1]<0:
                desiredAngle=2*np.pi-desiredAngle

        else:
            desiredAngle=desiredAngle
        print('Heading angle: ', heading)
        print('Desired angle: ', desiredAngle)
        steer_angle = K*(desiredAngle-heading)

        if steer_angle<-self.max_steer_angle:
            steer_angle=-self.max_steer_angle
        if steer_angle>self.max_steer_angle:
            steer_angle=self.max_steer_angle

        print('Steer angle: ', steer_angle)

        self.steer_angle = steer_angle


    def Drive(self, realPos, desPosarr, heading):
        self.nextDesPos(realPos, desPosarr)
        self.updateSteering(realPos, heading)
        self.updateVelocity()
        motor_velocity = self.motor_velocity
        steer_angle = self.steer_angle
        return motor_velocity, steer_angle
