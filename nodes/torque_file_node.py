import rospy
from mobile_hdemg_exo.msg import StampedFloat64
from mobile_hdemg_exo.streamer.torque_file_streamer import TorqueFileStreamer
from mobile_hdemg_exo.utils.moving_average import MovingAverage
from std_msgs.msg import Float64

SAMPLING_FREQUENCY = rospy.get_param("/sampling_frequency", int)
SCALE_FACTOR = 20  # Makes it easier to visualize EMG on the same plot

while not rospy.get_param("startup_gui_completed"):
    rospy.sleep(0.1)


class TorqueFileNode:
    """
    A ROS node for streaming torque data from a recorded dataset.
    """

    def __init__(self):
        rospy.init_node('torque_file_node')
        self.r = rospy.Rate(SAMPLING_FREQUENCY)
        self.start_time = rospy.Time.now()
        self.torque_pub = rospy.Publisher(
            'file_torque', StampedFloat64, queue_size=10)
        self.path = rospy.get_param(
            "/file_dir") + "/data/test_data_nov_dec/sample_trap_torque.csv"
        self.streamer = TorqueFileStreamer(SAMPLING_FREQUENCY, self.path)
        self.mov_avg = MovingAverage(5*SAMPLING_FREQUENCY)  # 5 second window

    def run(self):
        """
        Streams torque data from a recorded dataset.
        """
        torque_message = StampedFloat64()
        torque_message.header.stamp = rospy.Time.now() - self.start_time
        torque_data = abs(self.streamer.stream_data()) / SCALE_FACTOR
        smooth_torque = self.mov_avg.moving_avg(torque_data)
        torque_message.data = Float64(smooth_torque)
        self.torque_pub.publish(torque_message)
        self.r.sleep()


if __name__ == '__main__':
    torque_file_node = TorqueFileNode()
    while not rospy.is_shutdown():
        torque_file_node.run()
