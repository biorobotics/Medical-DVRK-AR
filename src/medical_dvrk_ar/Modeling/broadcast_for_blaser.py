#!/usr/bin/env python  
import roslib

import rospy
import tf

if __name__ == '__main__':
	
	rospy.init_node('tf_broadcaster_for_blaser')
	br = tf.TransformBroadcaster()

	rate = rospy.Rate(10.0)

	while not rospy.is_shutdown():

		br.sendTransform((0, 0.01561, 0.02755),
						 (0.0, 0.0, 0.0, 1.0),
						 rospy.Time.now(),
						 "blaser",
						 "/PSM1_tool_wrist_sca_shaft_link")
		rate.sleep()
