#!/usr/bin/env python3

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import numpy
def talker():
    pub = rospy.Publisher('floats', numpy_msg(Floats),queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(512) # 10hz
    while not rospy.is_shutdown():
        
        a=numpy.array(numpy.ones((64,1)), dtype=numpy.float32)
        a=a.flatten()
        pub.publish(a)
        r.sleep()

if __name__ == '__main__':
    talker()