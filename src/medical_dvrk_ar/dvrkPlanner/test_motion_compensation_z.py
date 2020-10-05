#!/usr/bin/env python
import rospy
from dvrk import psm
import PyKDL
import math
from std_msgs.msg import Float64

class unit_test:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.robot = psm('PSM1')
        self.robot.home()
        position_start = self.robot.get_current_position()

        self.set = False
        self.old_move = 0
        self.amp = 0.05
        self.freq = 0.5
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
        # Get blaser vectors
            run_time = rospy.Time.now().to_sec()
            offset_z = self.amp * math.sin(self.freq * run_time)
            self.robot.dmove(PyKDL.Vector(0, 0, offset_z - self.old_move),blocking=False)
            self.old_move = offset_z
            rate.sleep()



if __name__ == '__main__':
    test_z = unit_test()
    
