#!/usr/bin/env python
"""
Function:
add blaser frame to the tf tree

Author:
Cora

Date:
May 1

"""

import roslib
import numpy
import rospy
import tf

if __name__ == '__main__':
	"""
	The result of blaser-PSM1_tool_wrist_sca_link registration is
	- [-0.9968694040616278, 0.07188731680203468, -0.032918155003256816, 0.028613107797822702]
	- [0.06447174780333857, 0.4980597621877277, -0.8647426594222628, 0.3233587156450257]
	- [-0.04576882105754135, -0.8641577905524593, -0.5011352392782755, -0.00948986780330055]
	- [0.0, 0.0, 0.0, 1.0]
	"""
	
	rospy.init_node('tf_broadcaster_for_blaser')
	br = tf.TransformBroadcaster()
	rot_matrix = np.array([[-0.9968694040616278, 0.07188731680203468, -0.032918155003256816],
						[0.06447174780333857, 0.4980597621877277, -0.8647426594222628],
						[-0.04576882105754135, -0.8641577905524593, -0.5011352392782755]])
	rot_quaternion = tf.transformations.quaternion_from_matrix(rot_matrix)

	rate = rospy.Rate(10.0)

	while not rospy.is_shutdown():

		br.sendTransform((0.028613107797822702, 0.3233587156450257, -0.00948986780330055),
						 (rot_quaternion[0], rot_quaternion[1], rot_quaternion[2], rot_quaternion[3]),
						 rospy.Time.now(),
						 "blaser",
						 "/PSM1_tool_wrist_sca_link")
		rate.sleep()
