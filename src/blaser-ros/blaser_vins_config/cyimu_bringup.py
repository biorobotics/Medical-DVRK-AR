#!/usr/bin/env python
import serial
import struct
import sys

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np

rospy.init_node('hither')
portname = rospy.get_param('~port', '/dev/ttyACM0')
topic = rospy.get_param('~topic', '/imu')

print(portname)
print(topic)

s = serial.Serial(portname)
#s = serial.Serial('/dev/serial/by-id/usb-Cypress_Semiconductor_USBUART_810302070F0A1164-if00')
pub = rospy.Publisher(topic, Imu, queue_size=1);

gain = 1
nbit = 10
l = s.read(4*nbit)
vm = np.array(struct.unpack('<'+'f'*nbit,l))

while not rospy.is_shutdown():
    l = s.read(4*nbit)
    vs = struct.unpack('<'+'f'*nbit,l)
    #print(('%9.5f '*nbit) % vs)

    vm = (1-gain)*vm + gain*np.array(vs)

    imu = Imu()
    imu.header.stamp = rospy.Time.now()
    imu.orientation.x = vm[0]
    imu.orientation.y = vm[1]
    imu.orientation.z = vm[2]
    imu.orientation.w = vm[3]

    imu.linear_acceleration.x = vm[4]
    imu.linear_acceleration.y = vm[5]
    imu.linear_acceleration.z = vm[6]

    imu.angular_velocity.x = vm[7]
    imu.angular_velocity.y = vm[8]
    imu.angular_velocity.z = vm[9]

    pub.publish(imu)

