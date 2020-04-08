import rospy
from dvrk import psm
import PyKDL
import numpy as np

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

def resetZRot(robot):
        curr = robot.get_current_joint_position()
        robot.move_joint(np.array([curr[0],
                                   curr[1],
                                   curr[2],
                                   0,
                                   curr[4],
                                   curr[5]]))

def main():
    rospy.init_node('controller', anonymous=True)
    robot = psm('PSM1')
    rate = rospy.Rate(10000) # 10hz
    robot.home()
    position_start = robot.get_current_position()
    print(position_start.p+PyKDL.Vector(1,1,1))

    safeSpot = PyKDL.Frame()
    safeSpot.p = PyKDL.Vector(0.05,-0.04,-0.13)
    safeSpot.M = fingertipConstraint(safeSpot.p)
    robot.move(safeSpot)
    resetZRot(robot)
    rate.sleep()

    step_val_x = 0.004
    step_val_y = 0.001
    step_val_z = 0.0001

    dir = 1
    z_dir = 1

    for i in range(50):
        for j in range(90):
            if j > 45:
                z_dir = -1
            robot.dmove(PyKDL.Vector(0, step_val_y*dir, step_val_z*z_dir))
        robot.dmove(PyKDL.Vector(-step_val_x, 0, 0))
        dir *= -1
        z_dir = 1
    while not rospy.is_shutdown():
        print("Scan complete")
        rate.sleep()

    while not rospy.is_shutdown():
        print("Scan complete")
        rate.sleep()
if __name__ == '__main__':
    main()