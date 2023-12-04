import rospy
from std_msgs.msg import Float64MultiArray
from mobile_hdemg_exo.msg import StampedFloat64MultiArray
from mobile_hdemg_exo.utils.moving_average import MovingAverage
from mobile_hdemg_exo.utils.processing import notch_filter, butter_bandpass, csv_output
from mobile_hdemg_exo.processors.emg_process_cst import EMGProcessorCST
import numpy as np
from scipy import signal

while not rospy.get_param("startup_gui_completed"):
    rospy.sleep(0.1)

SAMPLING_FREQUENCY = rospy.get_param("/sampling_frequency", int)
MUSCLE_COUNT = rospy.get_param("/muscle_count", int)


class EMGProcessorNode:
    """
    A class for processing EMG data.

    Attributes:
        r: A ROS Rate object.
        raw_sub: A ROS subscriber for the /hdEMG_stream_raw topic.
        processed_pub: A ROS publisher for the /hdEMG_stream_processed topic.
        raw_data: A list of integers representing EMG data.
        raw_timestamp: The timestamp of the EMG data.
        moving_avg_object: A MovingAverage object.
        start_time: The time at which the node was started.

    Methods:
        callback: A callback function for the /hdEMG_stream_raw topic.
        process_emg: Processes EMG data.
        notch_filter: Applies a notch filter to the EMG data.
        butter_bandpass: Creates a bandpass filter.
        csv_output: Outputs the EMG reading to a CSV file.
    """

    def __init__(self):
        rospy.init_node('emg_processor_node')
        self.r = rospy.Rate(SAMPLING_FREQUENCY)
        self.start_time = rospy.get_time()
        self.raw_sub = rospy.Subscriber(
            'hdEMG_stream_raw', StampedFloat64MultiArray, self.callback)
        self.processor = EMGProcessorCST()
        self.rms_pub = rospy.Publisher(
            'hdEMG_stream_rms', StampedFloat64MultiArray, queue_size=10)
        self.cst_pub = rospy.Publisher(
            'hdEMG_stream_cst', StampedFloat64MultiArray, queue_size=10)
        self.raw_data = None
        self.cst_array = []
        self.rms_array = []
        self.rms_mov_avg = MovingAverage(
            window_size=SAMPLING_FREQUENCY)
        self.cst_mov_avg = MovingAverage(
            window_size=SAMPLING_FREQUENCY)
        self.b, self.a = butter_bandpass(20, 200, SAMPLING_FREQUENCY, order=2)

    def callback(self, raw_message):
        self.raw_data = raw_message.data.data

    def process_emg(self):
        notch_reading = notch_filter(self.raw_data, SAMPLING_FREQUENCY)
        hdemg_filtered = signal.filtfilt(self.b, self.a, notch_reading)
        removed_channels_param = rospy.get_param("/channels_to_remove", str)
        if removed_channels_param != '':
            removed_channels = [int(channel)
                                for channel in removed_channels_param.split(",")]
        else:
            removed_channels = []
        for channel in removed_channels:
            hdemg_filtered[channel] = 0.0
        # RMS
        self.rms_array = []
        for muscle in range(MUSCLE_COUNT):
            muscle_emg = hdemg_filtered[muscle*64:(muscle+1)*64]
            rms_emg = (np.mean(np.array(muscle_emg)**2))**0.5
            self.rms_array.append(rms_emg)
        rms_message = StampedFloat64MultiArray()
        rms_message.header.stamp = rospy.get_rostime().from_sec(
            rospy.get_time() - self.start_time)
        rms_message.data = Float64MultiArray(data=self.rms_array)
        self.rms_pub.publish(rms_message)
        # CST
        self.cst_array = self.processor.process_reading(hdemg_filtered/10)
        cst_message = StampedFloat64MultiArray()
        cst_message.header.stamp = rospy.get_rostime().from_sec(
            rospy.get_time() - self.start_time)
        cst_message.data = Float64MultiArray(data=self.cst_array)
        self.cst_pub.publish(cst_message)

        csv_output([self.rms_array, self.cst_array])
        self.r.sleep()


if __name__ == '__main__':
    emg_processor_node = EMGProcessorNode()
    while not rospy.is_shutdown():
        if emg_processor_node.raw_data is not None:
            emg_processor_node.process_emg()
