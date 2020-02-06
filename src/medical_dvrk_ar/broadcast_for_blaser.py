#!/usr/bin/env python  
import roslib

import rospy
import tf

if __name__ == '__main__':
	
	rospy.init_node('cora_broadcaster_for_blaser')
	br = tf.TransformBroadcaster()

	rate = rospy.Rate(10.0)

	while not rospy.is_shutdown(): 

		br.sendTransform((20.0, 20.0, 20.0),
						 (0.0, 0.0, 0.0, 1.0),
						 rospy.Time.now(),
						 "carrot1",
						 "/PSM1_tool_wrist_sca_shaft_link")

		rate.sleep()