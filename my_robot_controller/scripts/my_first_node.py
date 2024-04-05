


#!/usr/bin/env python3 
import rospy
from geometry_msgs.msg import Twist
import tensorflow  as tf
from tensorflow import keras

















import numpy as np
from keras import Sequential
import rospkg, os
from utils import ITD
import scipy
from scipy import signal 
from rospy_tutorials.msg import Floats
from matplotlib.pyplot import xcorr
import matplotlib.pyplot as plt
firstIndex=0
secondIndex=0
epoch_size=1536
importantChannels=[14,13,12,10,18,48,49,50,46,56]
vector=np.empty((64,0))
pub=rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=10)
sos = signal.butter(30, (8,30), 'bandpass', fs=512, output='sos')
itd = ITD()
def bandpower(x, fs, fmin, fmax):
    f, Pxx = signal.periodogram(x, fs=fs)
    ind_min = scipy.argmax(f > fmin) - 1
    ind_max = scipy.argmax(f > fmax) - 1
    return scipy.trapz(Pxx[ind_min: ind_max], f[ind_min: ind_max])
def spectral_entropy(x, fs, instant=True):
    f, Pxx = signal.welch(x, fs=fs, nperseg=len(x)//30, nfft=len(x)//30)
    Pxx_normalized = Pxx / np.sum(Pxx)
    
    spectral_entropies = -(Pxx_normalized * np.log2(Pxx_normalized + 1e-12))
    
    if instant:
        return spectral_entropies[:30]



def metricsExtractor(input):
    output=np.zeros(1,32)
    output[0]=xcorr(input,input)
    output[1]=bandpower(input,512,8,30)
    output[2:]=spectral_entropy(input,512)
    return output
#filtered = signal.sosfilt(sos, sig)
def preprocessing(vector):
    output=np.zeros((10,3,32))
    for i in range(vector.shape[0]):
        vector[i,:]=signal.sosfilt(sos, vector[i,:])
        vector[i,:]=vector[i,:]-np.mean(vector[i,:])
    for i in range(vector.shape[1]):
        for e in importantChannels:
            vector[e,i]=vector[e,i]-np.mean(vector[:,i])
    plt.figure(figsize=(10, 5))        
    print( vector[0,:]
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    )
    for i in importantChannels :
        print(vector[i,:].shape)
        rotations = itd.itd(vector[i,:])
        output[i][0][:]=metricsExtractor(rotations[0,:])
        output[i][1][:]=metricsExtractor(rotations[1,:])
        output[i][2][:]=metricsExtractor(itd.get_baselines())
    return output
# for i=1:1:64;signRf(i,:)=filter(filtroPB,signR(i,:));end
# for i=1:64
# signLf(i,:)=signLf(i,:)-mean(signLf(i,:));
# signRf(i,:)=signRf(i,:)-mean(signRf(i,:));
# end
# %% common average

# t=0:Ts:(length(signLf)-1)*Ts;
# figure
# plot(t(1:end),signRf(:,1:end));title('Time plot filtered signal imagery left');
# xlabel ('Time(s)');ylabel ('Amplitude(V)');
# figure
# plot(t(1:end),signRf(:,1:end)); title('Time plot filtered signal imagery right');
# xlabel ('Time(s)');ylabel ('Amplitude(V)');
# % Split data
# % disp(num-limitTrain)

# importantChannels=[14,13,12,10,18,48,49,50,46,56]


# num=epoch_size*(epoch_number-length(daRimuovere));
# for i =1:epoch_size
#     for e= importantChannels
#     signRf(e,i)=signRf(e,i)-mean(signRf(:,i));
#     signLf(e,i)=signLf(e,i)-mean(signLf(:,i));

#     end
# end
def callback(data):
    global vector
    #print (rospy.get_name(),type(data.data), data.data)
    print(vector.shape)
    vector=np.concatenate((vector,np.array(data.data).reshape(64,1)),axis=1)
    print(vector.shape[1])
    if vector.shape[1]>1535:
        vectorTemp=preprocessing(vector)
        vectorTemp=vector.reshape(1,8,3,32)
        vector=np.empty((64,0))
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

