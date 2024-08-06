#!/usr/bin/env python3 
import rospy
from geometry_msgs.msg import Twist
import tensorflow  as tf
from tensorflow import keras
import numpy as np
from keras import Sequential
import rospkg, os
import utils
import scipy.io

from sklearn.neighbors import KNeighborsClassifier
from sklearn.metrics import accuracy_score

from rospy_tutorials.msg import Floats
firstIndex=0
secondIndex=0
vector=np.random.rand(8,3,32)
pub=rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=10)
print(tf.version.VERSION)
def callback(data):
    #print (rospy.get_name(),type(data.data), data.data)
    vector=np.array(data.data)
    #print(vector)
    vectorTemp=vector.reshape(1,14,32)
    prediction=model.predict(vectorTemp)
    prediction=knn.predict(prediction)
    prediction=np.argmax(prediction,axis=1)
    print(prediction)
    if(prediction==0):
        msg=Twist()
        msg.linear.x=2.0
        msg.angular.z=1.0
        pub.publish(msg)
    elif prediction==1:
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
    rospy.Subscriber("elaborated", Floats, callback)
    rospy.spin()

rospack = rospkg.RosPack()
path=os.path.join(rospack.get_path("my_robot_controller"),"modelEmotiv.keras")
model=tf.keras.models.load_model(path) 
time_step =[]
temps = []
temporanea=[]


dataset = scipy.io.loadmat(os.path.join(rospack.get_path("my_robot_controller"),'datasetMarco.mat'))#
dataset=dataset["newMatrix"]#["ExportMatrix"]#
print(len(dataset))


data1=np.array(dataset)
print(data1.shape)

for row in range(len(dataset)) :
    for i in range(3):

        if(i==0):
            time_step.append([1,0,0])
        elif(i==1):
            time_step.append([0,1,0])
        else:
            time_step.append([0,0,1])
        temps.append(dataset[row,i,:,:])
series = np.array(temps)
time = np.array(time_step)
indices = np.arange(series.shape[0])
np.random.shuffle(indices)
print(series.shape)
support_features = model.predict(series[:27])
# predictions=np.argmax(predictions,axis=1)
# print(len(predictions))
# print(np.argmax(time,axis=1))
y_test=time
support_labels=y_test[:27]

# Train k-NN classifier
knn = KNeighborsClassifier(n_neighbors=3)  # You can adjust the number of neighbors
knn.fit(support_features, support_labels)

# Predict labels for the support set
support_predictions = knn.predict(support_features)
query_features=model.predict(series[27:])
# Predict labels for the query set
query_predictions = knn.predict(query_features)

query_labels=y_test[27:]
# Calculate accuracy for the support set
support_accuracy = accuracy_score(support_labels, support_predictions)
print(f'Support Set Accuracy: {support_accuracy * 100:.2f}%')
print(query_labels)
# Calculate accuracy for the query set
query_accuracy = accuracy_score(np.argmax(query_labels,axis=1), np.argmax(query_predictions,axis=1))
print(f'Query Set Accuracy: {query_accuracy * 100:.2f}%')    
if __name__=="__main__":
    listener()
    

    rospy.loginfo("finish")
