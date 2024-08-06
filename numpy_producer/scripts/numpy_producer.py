#!/usr/bin/env python3

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import numpy as np
import mne
def gdf_data(file_path):
    raw = mne.io.read_raw_gdf(file_path, preload=True)
    data, times = raw[:, :] # ottiene tutti i dati
    data = np.array(data, dtype=np.float32)
    return data, times, raw.info


def talker():
    gdf_file_path = '/home/marco/Data/Calibrazione.gdf'
    raw_data, times, info = gdf_data(gdf_file_path)
    total_samples = raw_data.shape[1]
    index = 0
    print(raw_data.shape)
    pub = rospy.Publisher('floats', numpy_msg(Floats),queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(512) # 10hz
    while not rospy.is_shutdown():
        end_index = index 
        if end_index < total_samples:
            # Estrai i dati per il frame corrente
            sample_data = raw_data[:, index]
        else:
            # Riavvia il buffer se i dati sono finiti
            index = 0
            continue
        index=index+1
        a=np.array(sample_data, dtype=np.float32)
        a=a.flatten()
        pub.publish(a)
        r.sleep()

if __name__ == '__main__':
    talker()
