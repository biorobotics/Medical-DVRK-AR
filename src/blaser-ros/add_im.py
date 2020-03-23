import rospy
import tf
import sys
import cv2
import os
import glob

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# add_im.py is a convenience script to collect data in the right format for the matlab calibration.
# Usage: python add_im.py path/to/dir
#  This save directory must be empty to begin with, and should only contain the txt descriptor file and the images.
#  Saves images and ur5 poses.

if __name__ == '__main__':
    rospy.init_node('calib_im_adder')

    if len(sys.argv) <= 1:
        print('Need save dir')
        print('Usage: python add_im.py /path/to/save/dir')
        exit(1)

    path = sys.argv[1]
    if not path.endswith('/'):
        path = path+'/'
    if not os.path.exists(path):
        os.makedirs(path)


    ls = tf.TransformListener()

    trans, rot = 0, 0

    print('Waiting for transform...')
    while not rospy.is_shutdown():
        try:
            (trans, rot) = ls.lookupTransform('/base_link', '/ee_link', rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException):
            continue

    print('Waiting for image...')
    msg = rospy.wait_for_message('/blaser_camera/image_color', Image, 1)
    img = CvBridge().imgmsg_to_cv2(msg, 'bgr8')

    print('got both!')

    f1 = 'data.txt'
    n_im = len(glob.glob(path + '*.png'))
    f2 = 'im{}.png'.format(n_im)

    print(f1)
    print(f2)
    print(trans)
    print(rot)

    with open(path+f1, 'a+') as f:
        f.write('{}\n{}\n{}\n'.format(trans, rot, f2))
    cv2.imwrite(path+f2, img)
    print('done!')
