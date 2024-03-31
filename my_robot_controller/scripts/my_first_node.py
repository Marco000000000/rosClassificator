#!/usr/bin/env python3 
import rospy
from geometry_msgs.msg import Twist
import tensorflow  as tf
from tensorflow import keras
import numpy as np
from keras import Sequential
import rospkg, os
import utils
from rospy_tutorials.msg import Floats
firstIndex=0
secondIndex=0
vector=np.random.rand(8,3,32)
pub=rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=10)

def callback(data):
    #print (rospy.get_name(),type(data.data), data.data)
    vector=np.array(data.data)
    vectorTemp=vector.reshape(1,8,3,32)
    if(model.predict(vectorTemp)[0]>0.5):
        msg=Twist()
        msg.linear.x=2.0
        msg.angular.z=1.0
        pub.publish(msg)
    else:
        msg=Twist()
        msg.linear.x=-2.0
        msg.angular.z=1.0
        pub.publish(msg)

# def callback(data):
#     global pub
#     global vector
#     global firstIndex
#     global secondIndex
#     print (rospy.get_name(),type(data.data), data.data)
#     vector=np.array(data.data)
#     print("firstIndex",firstIndex,"secondIndex",secondIndex)
#     if secondIndex==2:
#         secondIndex=0
#         if firstIndex==7:
#             firstIndex=0
#             secondIndex=0
#             vectorTemp=vector.reshape(1,8,3,32)
#             if(model.predict(vectorTemp)[0]>0.5):
#                 msg=Twist()
#                 msg.linear.x=2.0
#                 msg.angular.z=1.0
#                 pub.publish(msg)
#             else:
#                 msg=Twist()
#                 msg.linear.x=-2.0
#                 msg.angular.z=1.0
#                 pub.publish(msg)
#             return
#         firstIndex=firstIndex+1
        
#     else:
#         secondIndex=secondIndex+1    
def listener():
    rospy.init_node('listener')
    rospy.Subscriber("floats", Floats, callback)
    rospy.spin()
 
rospack = rospkg.RosPack()
path=os.path.join(rospack.get_path("my_robot_controller"),"modelOverfitted.keras")
model=keras.models.load_model(path) 
if __name__=="__main__":
    listener()


    rospy.loginfo("finish")

