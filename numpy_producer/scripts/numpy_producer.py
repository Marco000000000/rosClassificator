#!/usr/bin/env python3

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import numpy
def talker():
    pub = rospy.Publisher('floats', numpy_msg(Floats),queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        
        a=numpy.array(numpy.random.rand(1,8,3,32), dtype=numpy.float32)
        a=a.flatten()
        pub.publish(a)
        r.sleep()

if __name__ == '__main__':
    talker()