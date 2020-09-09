#!/usr/bin/env python 
import rospy
from dvrk import psm
import PyKDL
import numpy as np
from tf_conversions import posemath

import os.path
functionPath = os.path.dirname(os.path.realpath(__file__))

# home:
# [[-2.69849e-11,           1,-9.91219e-17;
#             1, 2.69849e-11, 7.34641e-06;
#   7.34641e-06, 9.91194e-17,          -1]
# [ 4.16909e-07, 4.50335e-07,     -0.1135]]

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


def arrayToPyKDLRotation(array):
    x = PyKDL.Vector(array[0][0], array[1][0], array[2][0])
    y = PyKDL.Vector(array[0][1], array[1][1], array[2][1])
    z = PyKDL.Vector(array[0][2], array[1][2], array[2][2])
    return PyKDL.Rotation(x,y,z)

def arrayToPyKDLFrame(array):
    rot = arrayToPyKDLRotation(array)
    pos = PyKDL.Vector(array[0][3],array[1][3],array[2][3])
    return PyKDL.Frame(rot,pos)

def fingertipConstraint(vectorDesired):
    ''' Find a PyKDL rotation from a vector using taylor series expansion
    '''
    vector = vectorDesired / vectorDesired.Norm()
    # Cross product of vectorDesired with z vector
    v = PyKDL.Vector(-vector.y(), vector.x(), 0)
    s = v.Norm()

    if s == 0:
        retval = PyKDL.Rotation(vector.z(), 0, 0,
                                0, 1, 0,
                                0, 0,vector.z())
        retval.DoRotZ(np.pi/2)
        return retval


    skew = np.matrix([[   0.0, -v.z(),  v.y()],
                      [ v.z(),    0.0, -v.x()],
                      [-v.y(), v.x(),    0.0]])
    c = vector.z()
    R = np.eye(3) + skew + skew*skew*(1-c)/(s*s);

    kdlRotation = arrayToPyKDLRotation(R.tolist())
    z, y  = kdlRotation.GetEulerZYZ()[0:2]
    #print(z,y)
    retval = PyKDL.Rotation()
    retval.DoRotZ(z)
    retval.DoRotY(y)
    retval = PyKDL.Rotation.Rot(retval.UnitZ(), np.pi/2) * retval
    return retval


class ControlServer(object):
    def __init__(self, startp, endp):

        self.organPoseSub = rospy.Subscriber('registration_pose',
                                             PoseStamped, self.poseCb)
        self.organTransform = None
        self.robot = psm('PSM2')

        self.toolOffset = .02 # distance from pinching axle to center of orange nub

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

        # TODO make these not hard-coded
        self.maxDepth = 0.01 # Meters
        self.maxForce = 800 # Not sure of this?
        self.safeZ = .05 # Safe height above organ in meters
        self.normalDistance = 0.005 # Meters

        self.safeSpot = PyKDL.Frame()
        self.safeSpot.p = startp
        self.safeSpot.M = fingertipConstraint(startp)

        self.scanEndSpot = PyKDL.Frame()
        self.scanEndSpot.p = endp
        self.scanEndSpot.M = fingertipConstraint(endp)
        
        self.robot.move(self.safeSpot)
        self.resetZRot()
        self.rate.sleep()
        self.robot.move(self.scanEndSpot)


    def poseCb(self, data):
        self.organTransform = posemath.fromMsg(data.pose)

    def resetZRot(self):
        curr = self.robot.get_current_joint_position()
        self.robot.move_joint(np.array([curr[0],
                                        curr[1],
                                        curr[2],
                                        0,
                                        curr[4],
                                        curr[5]]))


    def move(self,desiredPose, maxForce):
        currentPose = self.robot.get_desired_position()
        currentPose.p = currentPose.p
        displacements = np.array([0], float)
        # Remove z rotation
        angle = np.arccos(PyKDL.dot(desiredPose.M.UnitX(), currentPose.M.UnitX()))
        rot = PyKDL.Rotation.Rot(desiredPose.M.UnitZ(), angle)

        # Added offset representing tooltip
        desiredPosition = desiredPose.p - desiredPose.M.UnitZ()*self.toolOffset
        desiredPoseWithOffset = PyKDL.Frame(desiredPose.M, desiredPosition)
        measuredPose_previous = self.robot.get_current_position()
        startForce = self.force[1]
        while not rospy.is_shutdown():
            # get current and desired robot pose (desired is the top of queue)
            # compute the desired twist "x_dot" from motion command
            xDotMotion = resolvedRates(self.resolvedRatesConfig,
                                       currentPose,
                                       desiredPoseWithOffset) # xDotMotion is type [PyKDL.Twist]
            currentPose = PyKDL.addDelta(currentPose,xDotMotion,self.resolvedRatesConfig['dt'])
            

            if xDotMotion.vel.Norm() <= 0.001 and xDotMotion.rot.Norm() <= 0.1:
                break

            self.robot.move(currentPose, interpolate = False)
            self.rate.sleep()
            measuredPose_current = self.robot.get_current_position()
            currentDisplacement = measuredPose_current.p-measuredPose_previous.p
            currentDisplacement = PyKDL.dot(currentDisplacement, desiredPose.M.UnitZ())

            displacements = np.append(displacements, [currentDisplacement])
        return displacements.tolist()

if __name__=="__main__":
    startp = PyKDL.Vector(0.06,0.00,-0.05)
    endp = PyKDL.Vector(-0.06,0.00,-0.05)
    rospy.init_node('Control_server')
    np.set_printoptions(precision=2)
    server = ControlServer()
