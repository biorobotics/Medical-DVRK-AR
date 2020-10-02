#!/usr/bin/env python 
import rospy
from dvrk import psm
import PyKDL
import numpy as np
from tf_conversions import posemath

import os.path
from point_estimation import estimation, make_PyKDL_Frame, estimation_numpy

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


class ControlServer(object):
    def __init__(self, amplitude, frequency):

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
        self.toolOffset = 0.05
        self.amp = amplitude
        self.freq = frequency

        # testing estimation
        self.start_time = rospy.Time.now().to_sec()

    def move(self,desiredPose, maxForce):
        # currentPose = self.robot.get_desired_position()

        # print(desiredPose.p)
        currentPose = self.robot.get_current_position()
        measuredPose_previous = self.robot.get_current_position()

        # # make a copy of the desiredPose
        # desiredPose_tmp = PyKDL.Frame()
        # pose = np.array([0.0,0.0,0.0])
        # pose[0] = np.copy(desiredPose.p[0])
        # pose[1] = np.copy(desiredPose.p[1])
        # pose[2] = np.copy(desiredPose.p[2])
        # rotation = np.copy(desiredPose.M)
        # desiredPose_tmp.p = PyKDL.Vector(pose[0],pose[1],pose[2])
        # desiredPose_tmp.M = desiredPose.M

        while not rospy.is_shutdown():
            # get current and desired robot pose (desired is the top of queue)
            # compute the desired twist "x_dot" from motion command
            # print(desiredPose.p)
            desiredPose_move = estimation(desiredPose, self.amp, self.freq, 0, rospy.Time.now().to_sec())
            # print(desiredPose_move.p)

            desiredPosition = desiredPose_move.p - desiredPose_move.M.UnitZ()*self.toolOffset
            
            desiredPoseWithOffset = PyKDL.Frame(desiredPose_move.M, desiredPosition)

            xDotMotion = resolvedRates(self.resolvedRatesConfig,
                                       currentPose,
                                       desiredPoseWithOffset) # xDotMotion is type [PyKDL.Twist]
            currentPose = PyKDL.addDelta(currentPose,xDotMotion,self.resolvedRatesConfig['dt'])
            

            if xDotMotion.vel.Norm() <= 0.001 and xDotMotion.rot.Norm() <= 0.1:
                break               
            self.robot.move(currentPose, interpolate = False)
            self.rate.sleep()

if __name__=="__main__":
    rospy.init_node('Control_server')

    # p3 = [-0.1 ,0.0, -0.1, 0.7071068,0.7071068,0,0]
    # p4=  [-0.05,0.0, -0.1, 0.7071068,0.7071068,0,0]
    # p5 = [0.0  ,0.0, -0.1, 0.7071068,0.7071068,0,0]
    # p6 = [0.05 ,0.0, -0.1, 0.7071068,0.7071068,0,0]
    # p7 = [0.1  ,0.0, -0.1, 0.7071068,0.7071068,0,0]
    # p8 = [0.05  ,0.0, -0.1, 0.7071068,0.7071068,0,0]
    # p9 = [0.0  ,0.0, -0.1, 0.7071068,0.7071068,0,0]
    # data = np.vstack((p3,p4,p5,p6,p7,p8,p9))
    # np.save('testdata.npy',data)
    data = np.load('testdata.npy')
    amplitude = 0.05 #0.05
    frequency = 0.5 #0.1
    server = ControlServer(amplitude, frequency)
    # server.move(make_PyKDL_Frame(data[0]), server.maxForce)

    number_of_points = len(data)
    for itr in range(0, number_of_points):
        print(itr)
        dest = make_PyKDL_Frame(data[itr])
        server.move(dest, server.maxForce)
