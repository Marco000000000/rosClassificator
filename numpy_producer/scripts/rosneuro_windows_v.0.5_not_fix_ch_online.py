#!/usr/bin/env python3

""" 
//////////////////////////////////////////////////////////////////////////////////////
////                        Author: Gregorio Amato                                ////
////                        Date: 31 July 2024 h: 00.41                           ////
////    Description: Implementation of the Acquisition from LSL to RosNeuro       ////
//////////////////////////////////////////////////////////////////////////////////////
"""
from pylsl import StreamInlet, resolve_stream
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from rosneuro_msgs.msg import NeuroFrame,NeuroDataFloat,NeuroDataInfo,NeuroDataInt32
from rosneuro_msgs.srv import GetAcquisitionInfo, GetAcquisitionInfoResponse
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import time
import sys
import mne
import threading

neuroseq=0
frameRate=256
samplerate= 256
device_id= 'WINDOWS'
device_model= 'EPOC_X'

# Variabili globali per i dati e il timestamp
stored_data = None
stored_timestamp = None

# Variabili globali per min e max dei dati
global_min = -sys.float_info.max
global_max = sys.float_info.max

"""non garantirebbe una copertura completa per il range dei dati
"""
#global_min = sys.float_info.min
#global_max = sys.float_info.max

# Cambia a True per usare la simulazione, False per file GDF
simulated_mode = True 
gdf_file_path = '/home/marco/Data/xx.20240806.143050.calibration.mi_bhbf.gdf'

# Numero di campioni per frame
num_samples_per_frame = samplerate // frameRate


class acquisition():
    def __init__(self):
        self.neuroseq_=0
        self.frame=None
        # Buffer per accumulare i dati
        self.data_buffer = []
        # Dati letti dal file GDF
        self.raw_data = [] 

acquisition = acquisition()

"""
NEURO_DATA
void NeuroData<T>::dump(void) {
        printf("[info] '%s' NeuroData:\n",	this->name().c_str());
        printf(" |- nsamples:\t\t%zu\n",	this->nsamples());
        printf(" |- nchannels:\t\t%zu\n",	this->nchannels());
        printf(" |- unit:\t\t%s\n",			this->info_.unit.c_str());
        printf(" |- transducter:\t%s\n",	this->info_.transducter.c_str());
        printf(" |- prefiltering:\t%s\n",	this->info_.prefiltering.c_str());
        printf(" |- min/max:\t\t[%f %f]\n", this->info_.minmax[0], this->info_.minmax[1]);
        printf(" |- labels:\t\t");
        for(auto it = this->info_.labels.begin(); it != this->info_.labels.end(); ++it)
            printf("%s ", (*it).c_str());
        printf("\n");
    }
"""
def newNeuroFrame(timestamp,data):
    global neuroseq, stored_data, stored_timestamp,global_min,global_max

    # Memorizza i dati e il timestamp
    stored_data = data
    stored_timestamp = timestamp

    # Calcola min e max per i dati correnti
    local_min = np.min(data)
    local_max = np.max(data)

    # Aggiorna i valori globali min e max
    global_min = min(global_min, local_min)
    global_max = max(global_max, local_max)

    frame=NeuroFrame()
    frame.header.seq=neuroseq
    frame.header.stamp=rospy.get_rostime()
    frame.neuroheader.seq=neuroseq
    neuroseq+=1
    frame.sr=samplerate
    
    # Calcola il numero di canali basato sulla dimensione dei dati filtrati
    nchannels = len(data)
    # Assicurati che i dati siano float
    data = np.array(data, dtype=np.float32)
    
    frame.eeg=NeuroDataFloat()
    frame.eeg.data=data[:-1].flatten()
    #frame.eeg.data=data.flatten()
    frame.eeg.info=NeuroDataInfo()
    frame.eeg.info.nchannels= nchannels
    frame.eeg.info.unit="uV"
    frame.eeg.info.transducter="n/a"
    frame.eeg.info.prefiltering="n/a"

    frame.eeg.info.nsamples= num_samples_per_frame
    frame.eeg.info.stride= nchannels * np.float32().itemsize
    frame.eeg.info.labels=["AF3","F7","F3","FC5","T7","P7","O1","O2","P8","T8","FC6","F4","F8","AF4"]
    #frame.eeg.info.minmax=[]
    #frame.eeg.info.minmax.append(sys.float_info.min)
    #frame.eeg.info.minmax.append(sys.float_info.max)
    frame.eeg.info.minmax = [global_min, global_max]
    frame.eeg.info.isint=0

    # Aggiungi gli altri dati se necessario (exg e tri)
    frame.exg = NeuroDataFloat()
    frame.tri = NeuroDataInt32()

    return frame

def get_acquisition_info(req):
    response = GetAcquisitionInfoResponse()
    response.result = True
    response.device_model = device_model
    response.device_id = device_id
    
    # Usa i dati e il timestamp memorizzati
    if stored_data is not None and stored_timestamp is not None:
        response.frame = newNeuroFrame(stored_timestamp, stored_data)
    else:
        # Se non ci sono dati memorizzati, restituisci un frame vuoto o di default
        response.frame = NeuroFrame()
    
    return response

""" Non funziona !!! ma è inutilte perche presente in rosneuro_recorder 
nonavvia il recorder, problema in acquisizione
def start_recording():
    try:
        rospy.wait_for_service('recorder/record')
        record_service = rospy.ServiceProxy('recorder/record', Empty)
        record_service()
        rospy.loginfo("Recording started")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call start recording service: %s" % e)

def quit_recording():
    try:
        rospy.wait_for_service('/recorder/quit')
        quit_service = rospy.ServiceProxy('/recorder/quit', Empty)
        quit_service()
        rospy.loginfo("Recording stopped")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call stop recording service: %s" % e)
"""

# NeuroFrame Message

# Header
# Header header uint32 seq time stamp string frame_id


# NeuroHeader
#NeuroHeader neuroheader  uint32 seq

# Sampling rate
#uint16 sr

# NeuroData
#NeuroDataFloat eeg
#NeuroDataFloat exg
#NeuroDataInt32 tri

# GetAcquisitionInfo.srv
#---
#bool            result
#string device_model
#string device_id
#NeuroFrame frame

def acquisition_info_service():
    rospy.loginfo("Waiting for the acquisition info service to be ready...")
    rospy.Service('/acquisition/get_info', GetAcquisitionInfo, get_acquisition_info)
    rospy.wait_for_service('/acquisition/get_info')
    rospy.loginfo("Acquisition Info Service is ready.")
    rospy.spin()

def gdf_data(file_path):
    raw = mne.io.read_raw_gdf(file_path, preload=True)
    data, times = raw[:, :] # ottiene tutti i dati
    data = np.array(data, dtype=np.float32)
    return data, times, raw.info

def main():
    rospy.init_node('rosneuro_windows')
    r = rospy.Rate(frameRate)
    pub = rospy.Publisher('/neurodata', NeuroFrame,queue_size=10)
    
    # Avvia il servizio di thread
    service_thread = threading.Thread(target=acquisition_info_service)
    service_thread.start()

    if simulated_mode:
        print("Simulated mode is ON.")
        # Carica i dati dal file GDF per la simulazione
        raw_data, times, info = gdf_data(gdf_file_path)
        total_samples = raw_data.shape[1]
        index = 0

        #start_recording()

        while not rospy.is_shutdown():
            # Calcola il numero di campioni da prendere
            end_index = index + num_samples_per_frame
            if end_index <= total_samples:
                # Estrai i dati per il frame corrente
                sample_data = raw_data[:, index:end_index]
            else:
                # Riavvia il buffer se i dati sono finiti
                index = 0
                continue
            
            # Crea e pubblica il frame
            frame = newNeuroFrame(time.time(), sample_data)
            pub.publish(frame)
            #print("Published frame:", frame)
            r.sleep()
            
            index += num_samples_per_frame   
        #quit_recording()  
    else :
        # first resolve an EEG stream on the lab network
        
        print("looking for an EEG stream...")
        streams = resolve_stream('type', 'EEG')
        print(streams[0])
        
    # create a new inlet to read from the stream
        inlet = StreamInlet(streams[0])
        
        #start_recording()
        while not rospy.is_shutdown():
            sample, timestamp = inlet.pull_sample()
            #print(inlet.pull_sample())
            print(timestamp)
        
            # Converti i dati in un array numpy
            raw_data=np.array(sample, dtype=np.float32)
        
            # Aggiungi i dati al buffer
            acquisition.data_buffer.append(raw_data)
        
            # Se il buffer ha accumulato abbastanza campioni, crea e pubblica il frame
            if len(acquisition.data_buffer) >= num_samples_per_frame:
                # Prendi i primi num_samples_per_frame campioni
            
                data = np.vstack(acquisition.data_buffer[:num_samples_per_frame])
                acquisition.data_buffer = acquisition.data_buffer[num_samples_per_frame:]
                # Seleziona solo i 14 canali desiderati
                
                data = data[:, 3:-2].flatten()
                data = np.array(data, dtype=np.float32)
                
                # Crea un nuovo NeuroFrame
                frame = newNeuroFrame(timestamp, data)
                pub.publish(frame)
                
                print("Published frame:", frame)
            r.sleep()
        #quit_recording()

		# get a new sample (you can also omit the timestamp part if you're not
		# interested in it)
		
		#print(sample, timestamp)

if __name__ == '__main__':
    main()
