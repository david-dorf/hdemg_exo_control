#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from mobile_hdemg_exo.msg import StampedFloat64MultiArray
import numpy as np

while not rospy.get_param("startup_gui_completed"):
    rospy.sleep(0.1)

EMG_DEVICE = rospy.get_param("/device")
MUSCLE_COUNT = rospy.get_param("/muscle_count", int)
SAMPLING_FREQUENCY = rospy.get_param("/sampling_frequency", int)


class EMGStreamNode:
    """
    A class for streaming and processing EMG data.

    Attributes:
        r: A ROS Rate object.
        start_time: The time at which the node was started.
        emg_pub: A ROS publisher for the /hdEMG_stream_raw topic.
        p: A GPIO PWM object.
        streamer: An EMG streamer object.
        emg_offset: The offset of the EMG data in the raw stream.
        simulation_reading: A list of floats representing EMG data.
    """

    def __init__(self):
        """
        Initializes the EMGStreamNode object.

        Sets up the ROS publishers and initializes the EMG streamer and processor. Also sets up objects for latency analysis, smoothing, and simulation.
        """
        rospy.init_node('emg_stream_node')
        self.r = rospy.Rate(SAMPLING_FREQUENCY)
        self.start_time = rospy.get_time()
        self.emg_pub = rospy.Publisher(
            'hdEMG_stream_raw', StampedFloat64MultiArray, queue_size=10)

        if EMG_DEVICE == 'Quattrocento':
            from mobile_hdemg_exo.streamer.emg_qc_streamer import EMGQCStreamer
            self.streamer = EMGQCStreamer(MUSCLE_COUNT)
            self.emg_offset = 32 * MUSCLE_COUNT
        elif EMG_DEVICE == 'Muovi+Pro':
            from mobile_hdemg_exo.streamer.emg_muovi_streamer import EMGMUOVIStreamer
            self.streamer = EMGMUOVIStreamer(MUSCLE_COUNT)
            self.emg_offset = 0
        elif EMG_DEVICE == 'File':
            from mobile_hdemg_exo.streamer.emg_file_streamer import EMGFileStreamer
            self.path = rospy.get_param(
                "/file_dir") + "/data/test_data_nov_dec/sample_trap_emg.csv"
            self.streamer = EMGFileStreamer(
                MUSCLE_COUNT, SAMPLING_FREQUENCY, self.path)
            self.emg_offset = 0
        else:
            raise ValueError('Invalid EMG_DEVICE')

    def run_emg(self):
        """
        Reads EMG data from the streamer and publishes it to /hdEMG_stream_raw
        """
        raw_reading = self.streamer.stream_data()
        if len(raw_reading) < 64 * MUSCLE_COUNT:
            raw_reading = np.pad(raw_reading, (0, 64 * MUSCLE_COUNT - len(
                raw_reading)), 'constant', constant_values=(0, 0))
        hdemg_reading = raw_reading[self.emg_offset:
                                    self.emg_offset + 64 * MUSCLE_COUNT]
        raw_message = StampedFloat64MultiArray()
        raw_message.header.stamp = rospy.get_rostime().from_sec(
            rospy.get_time() - self.start_time)
        raw_message.data = Float64MultiArray(data=hdemg_reading)
        self.emg_pub.publish(raw_message)
        self.r.sleep()


if __name__ == '__main__':
    emg_stream_node = EMGStreamNode()
    while not rospy.is_shutdown():
        emg_stream_node.run_emg()
