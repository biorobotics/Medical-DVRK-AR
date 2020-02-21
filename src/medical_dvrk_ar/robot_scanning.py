#!/usr/bin/env python 
import rospy
from geometry_msgs.msg import Pose2D, PoseStamped, WrenchStamped
from force_sensor_gateway.msg import ForceSensorData
import yaml
from dvrk import psm
import PyKDL
import numpy as np
from dvrk_vision.uvtoworld import makeTexturedObjData
from dvrk_vision.uvtoworld import UVToWorldConverter
from tf_conversions import posemath
from dvrk_vision.clean_resource_path import cleanResourcePath
import force_sensor_gateway.ransac as ransac
from IPython import embed
from sensor_msgs.msg import RegionOfInterest
from IPython import embed

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


def arrayToPyKDLRotation(array):
    x = PyKDL.Vector(array[0][0], array[1][0], array[2][0])
    y = PyKDL.Vector(array[0][1], array[1][1], array[2][1])
    z = PyKDL.Vector(array[0][2], array[1][2], array[2][2])
    return PyKDL.Rotation(x,y,z)

def arrayToPyKDLFrame(array):
    rot = arrayToPyKDLRotation(array)
    pos = PyKDL.Vector(array[0][3],array[1][3],array[2][3])
    return PyKDL.Frame(rot,pos)

def rotationFromVector(vectorDesired):
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
    print(z,y)
    retval = PyKDL.Rotation()
    retval.DoRotZ(z)
    retval.DoRotY(y)
    retval = PyKDL.Rotation.Rot(retval.UnitZ(), np.pi/2) * retval
    return retval


class Probe2DServer(object):
    def __init__(self, cameraTransform, objPath, scale):

        self.organPoseSub = rospy.Subscriber('registration_pose',
                                             PoseStamped, self.poseCb)

        self.cameraTransform = cameraTransform
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
        self.safeSpot.p = PyKDL.Vector(0,0.00,-0.05)
        self.safeSpot.M = rotationFromVector(PyKDL.Vector(0,0,-.1))

        self.scanEndSpot = PyKDL.Frame()
        self.scanEndSpot.p = PyKDL.Vector(0,0.00,-0.05)
        self.scanEndSpot.M = rotationFromVector(PyKDL.Vector(0,0, .1))
        
        self.robot.move(self.safeSpot)
        self.resetZRot()
        self.rate.sleep()
        self.robot.move(self.scanEndSpot)


    def poseCb(self, data):
        self.organTransform = posemath.fromMsg(data.pose)

    def resetZRot(self):
        curr = self.robot.get_current_joint_position()
        # if(abs(curr[3]) > np.pi):
        #     print("RESETTING Z")
        self.robot.move_joint(np.array([curr[0],
                                        curr[1],
                                        curr[2],
                                        0,
                                        curr[4],
                                        curr[5]]))


    def move(self,desiredPose, maxForce):
        currentPose = self.robot.get_desired_position()
        currentPose.p = currentPose.p
        #forceArray = np.empty((0,4), float)
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
            # currentDisplacement =  xDotMotion.vel.Norm() * self.resolvedRatesConfig['dt']
            currentDisplacement = PyKDL.dot(currentDisplacement, desiredPose.M.UnitZ())
            # currentDisplacement = displacements[len(displacements)-1] + currentDisplacement
            #forceArray = np.append(forceArray, data, axis = 0)
            displacements = np.append(displacements, [currentDisplacement])
        return displacements.tolist()
        #return displacements.tolist(), forceArray.tolist()

if __name__=="__main__":
    rospy.init_node('probe_2D_server')
    yamlFile = cleanResourcePath("package://dvrk_vision/defaults/registration_params_cmu.yaml")
    with open(yamlFile, 'r') as stream:
        data = yaml.load(stream)
    cameraTransform = arrayToPyKDLFrame(data['transform'])
    np.set_printoptions(precision=2)
    # print np.matrix(data['transform'])
    # print cameraTransform.M
    meshPath = rospy.get_param("~mesh_path")
    scale = rospy.get_param("~scale")
    objPath = cleanResourcePath(meshPath)
    server = Probe2DServer(cameraTransform, objPath, scale)
