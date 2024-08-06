from pylsl import StreamInlet, resolve_stream
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
def main():
    # first resolve an EEG stream on the lab network
    print("looking for an EEG stream...")
    streams = resolve_stream('type', 'EEG')
    pub = rospy.Publisher('floats', numpy_msg(Floats),queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(512) # 10hz

    # create a new inlet to read from the stream
    inlet = StreamInlet(streams[0])

    while not rospy.is_shutdown():
        sample, timestamp = inlet.pull_sample()
        a=np.array(sample, dtype=np.float32)
        a=a.flatten()
        pub.publish(a)
        print(a.shape)

        r.sleep()
        # get a new sample (you can also omit the timestamp part if you're not
        # interested in it)
        
        #print(sample, timestamp)


if __name__ == '__main__':
    main()