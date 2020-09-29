#!/usr/bin/env python 
import rospy
from dvrk import psm
import PyKDL
import numpy as np
from tf_conversions import posemath

import os.path
functionPath = os.path.dirname(os.path.realpath(__file__))


def resolvedRates(config,currentPose,desiredPose):
    # compute pose error (result in kdl.twist format)
    poseError = PyKDL.diff(currentPose,desiredPose)
    posErrNorm = poseError.vel.Norm()
    rotErrNorm = poseError.rot.Norm()

    angVelMag = config['angVelMax']
    if rotErrNorm < config['tolRot']:
        angVelMag = 0.0
    elif rotErrNorm < angVelMag:
        angVelMag = rotErrNorm


    # compute velocity magnitude based on position error norm
    if posErrNorm>config['tolPos']:
        tolPosition = config['tolPos']
        lambdaVel = config['velRatio']
        if posErrNorm>(lambdaVel*tolPosition):
            velMag = config['velMax']
        else:
            velMax = config['velMax']
            velMin = config['velMin']
            velMag = velMin + (posErrNorm - tolPosition) * \
                     (velMax - velMin)/(tolPosition*(lambdaVel-1))
    else:
        velMag = 0.0
    # compute angular velocity based on rotation error norm
    if rotErrNorm>config['tolRot']:
        tolRotation = config['tolRot']
        lambdaRot = config['rotRatio']
        if rotErrNorm>(lambdaRot*tolRotation):
            angVelMag = config['angVelMax']
        else:
            angVelMax = config['angVelMax']
            angVelMin = config['angVelMin']
            angVelMag = angVelMin + (rotErrNorm - tolRotation) * \
                        (angVelMax - angVelMin)/(tolRotation*(lambdaRot-1))
    else:
        angVelMag = 0.0
    # The resolved rates is implemented as Nabil Simaan's notes
    # apply both the velocity and angular velocity in the error pose direction
    desiredTwist = PyKDL.Twist()
    poseError.vel.Normalize() # normalize to have the velocity direction
    desiredTwist.vel = poseError.vel*velMag
    poseError.rot.Normalize() # normalize to have the ang vel direction
    desiredTwist.rot = poseError.rot*angVelMag
    return desiredTwist
def make_PyKDL_Frame(point):
    pykdl_point = PyKDL.Frame()
    pykdl_point.p = PyKDL.Vector(point[0],point[1],point[2])
    pykdl_point.M = PyKDL.Rotation.Quaternion(point[3],point[4],point[5],point[6])
    return pykdl_point
    

class ControlServer(object):
    def __init__(self):

        self.robot = psm('PSM1')

        rate = 1000.0
        # TODO make these values not hard coded
        self.rate = rospy.Rate(rate) # 1000hz

        self.resolvedRatesConfig = \
        {   'velMin': 2.0/1000,
            'velMax': 30.0/1000,
            'angVelMin': 1.0/180.0*3.14,
            'angVelMax': 60.0/180.0*3.14,
            'tolPos': 0.1/1000.0, # positional tolerance
            'tolRot': 1.0/180.0*3.14, # rotational tolerance
            'velRatio': 1, # the ratio of max velocity error radius to tolarance radius, this value >1
            'rotRatio': 1,
            'dt': 1.0/rate, # this is the time step of the system. 
                            # if rate=1khz, then dt=1.0/1000. However, 
                            # we don't know if the reality will be the same as desired rate
        }

        self.maxDepth = 0.01 # Meters
        self.maxForce = 800 # Not sure of this?
        self.safeZ = .05 # Safe height above organ in meters
        self.normalDistance = 0.005 # Meters

    def move(self,desiredPose, maxForce):
        # currentPose = self.robot.get_desired_position()
        currentPose = self.robot.get_current_position()
        desiredPose_PyKDL = PyKDL.Frame(desiredPose.M, desiredPose.p)
        measuredPose_previous = self.robot.get_current_position()
        while not rospy.is_shutdown():
            # get current and desired robot pose (desired is the top of queue)
            # compute the desired twist "x_dot" from motion command

            '''
            TODO
            Here desiredPose_PyKDL should be updated costantly

            '''
            xDotMotion = resolvedRates(self.resolvedRatesConfig,
                                       currentPose,
                                       desiredPose_PyKDL) # xDotMotion is type [PyKDL.Twist]
            currentPose = PyKDL.addDelta(currentPose,xDotMotion,self.resolvedRatesConfig['dt'])
            

            if xDotMotion.vel.Norm() <= 0.001 and xDotMotion.rot.Norm() <= 0.1:
                # print('target pose: ')
                # print(currentPose)
                break               
            
            self.robot.move(currentPose, interpolate = False)
            self.rate.sleep()

if __name__=="__main__":
    rospy.init_node('Control_server')

    p1 = [0.1,0.02,-0.1,0.7071068,0.7071068,0,0]
    p2 = [0.01,0.02,-0.15, 0.7071068,0.7071068,0,0]
    p3 = [0.00,0.00,-0.2,0.7071068,0.7071068,0,0]
    data = np.vstack((p1,p2,p3))
    server = ControlServer()
    
    number_of_points = len(data)
    for itr in range(0, number_of_points):
        dest = make_PyKDL_Frame(data[itr])
        server.move(dest, server.maxForce)
