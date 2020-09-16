#!/usr/bin/env python
import rospy
from dvrk import psm
import PyKDL

def main():
    rospy.init_node('controller', anonymous=True)
    robot = psm('PSM1')
    rate = rospy.Rate(100) # 10hz
    robot.home()
    position_start = robot.get_current_position()
    safe_pos = PyKDL.Frame(PyKDL.Rotation(PyKDL.Vector(0, 1, 0),
                           PyKDL.Vector(1, 0, 0),
                           PyKDL.Vector(0, 0,-1)), 
                           PyKDL.Vector(0.05,-0.04,-0.13))
    robot.move(safe_pos)

    step_val_x = 0.02
    step_val_y = 0.005
    step_val_z = 0.0001
    dir = 1
    z_dir = 1

    for i in range(7):
        for j in range(18):
            if j > 45:
                z_dir = -1
            robot.dmove(PyKDL.Vector(0, step_val_y*dir, step_val_z*z_dir))
        robot.dmove(PyKDL.Vector(-step_val_x, 0, 0))
        dir *= -1
        z_dir = 1
    print("Scan complete")

if __name__ == '__main__':
    main()
