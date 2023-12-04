import numpy as np
import rospy
from h3_msgs.msg import State
from std_msgs.msg import Float64
from mobile_hdemg_exo.msg import StampedFloat64
from matplotlib import pyplot as plt
from mobile_hdemg_exo.calibration.trial import Trial, TrialDirection
import tkinter as tk
import pyttsx3
import rospy

# TODO: Multiple muscles and exoskeletons

EXO_TYPE = rospy.get_param("/exo_type", str)
MUSCLE_COUNT = rospy.get_param("/muscle_count", int)


class TrialRunner:
    _r: rospy.Rate
    _emg_sub: rospy.Subscriber
    _torque_sub: rospy.Subscriber
    _battery_sub: rospy.Subscriber
    _position_pub: rospy.Publisher
    trial: Trial

    def __init__(self, trial: Trial):
        self.trial = trial
        self._r = rospy.Rate(100)
        self._emg_array = []
        self._emg_time_array = []
        self._torque_array = []
        self._MVC_torque_array = []
        self._torque_time_array = []
        self._battery_voltage = []
        self.side = rospy.get_param("/side")
        self.device = rospy.get_param("/device")

        # EMG Setup
        self._cst_sub = rospy.Subscriber(
            '/hdEMG_stream_cst', StampedFloat64, self.emg_callback)
        self._rms_sub = rospy.Subscriber(
            '/hdEMG_stream_rms', StampedFloat64, self.emg_callback)

        if EXO_TYPE == "H3":
            # Enumerate the exoskeleton side for indexing the exo torque sensor array
            self.side_dict = {"Left": 5, "Right": 2, "File": 1}
            self._torque_sub = rospy.Subscriber(
                '/h3/robot_states', State, self.torque_callback)
            self._battery_sub = rospy.Subscriber(
                '/h3/robot_states', State, self.battery_callback)

            # Publisher for position control
            if (self.side == "Left"):
                self._position_pub = rospy.Publisher(
                    '/h3/left_ankle_position_controller/command', Float64, queue_size=0)
            elif (self.side == "Right"):
                self._position_pub = rospy.Publisher(
                    '/h3/right_ankle_position_controller/command', Float64, queue_size=0)
            elif (self.device == "File"):
                self._position_pub = rospy.Publisher(
                    '/h3/right_ankle_position_controller/command', Float64, queue_size=0)
            else:
                raise NameError(
                    "Side name must be Left, Right, or the system must be in File device mode")

        # Initialize the GUI
        self.window = tk.Tk()
        self.window.title(
            "Shirley Ryan AbilityLab - Patient Instruction for Ankle Exoskeleton")
        self.window.attributes('-fullscreen', True)
        self.message_label = tk.Label(
            self.window, font=("Calibri", 80), pady=20)
        self.message_label.pack(expand=True)

        # Initialize the text-to-speech engine
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 100)
        self.engine.setProperty('volume', 1.0)
        self.voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', self.voices[2].id)

    def emg_callback(self, data):
        self._emg_array.append(data.data.data)
        self._emg_time_array.append(data.header.stamp.to_sec())

    if EXO_TYPE == "H3":
        def torque_callback(self, data):
            self._torque_array.append(
                data.joint_torque_sensor[self.side_dict[self.side]])
            self._torque_time_array.append(data.header.stamp.to_sec())
            self._MVC_torque_array.append(
                data.joint_torque_sensor[self.side_id])

        def battery_callback(self, data):
            if data.battery_voltage < 18.0 and data.battery_voltage > 1:
                print("Please charge the battery" +
                      f"Battery voltage: {data.battery_voltage}")

        def _set_exo_angle(self, angle):
            print("Moving to {} degrees".format(str(np.rad2deg(angle))))
            self._position_pub.publish(float(angle))
            rospy.sleep(5)

    def collect_trial_data(self):
        if EXO_TYPE == "H3":
            self._set_exo_angle(self.trial.joint_angle)
        baseline_torque, min_torque = self._collect_baseline_torque()
        self.trial.baseline_torque = baseline_torque
        self.trial.min_torque = min_torque
        if self.trial.direction != TrialDirection.NoDirection:
            self.trial.MVC_torque = self._collect_max_torque()
        else:
            self.trial.MVC_torque = 2.0
        emg_coef = np.polyfit(
            self._emg_array, self._torque_array, 1)[0]
        rospy.set_param("emg_coef", emg_coef)
        rospy.set_param("calibrated", True)

    def update_gui(self, message):
        self.engine.say(message)
        self.engine.runAndWait()
        self.message_label.config(text=message)
        self.message_label.update()
        self.window.update()

    def _collect_baseline_torque(self):
        print("Collecting baseline torque...")
        self._MVC_torque_array = []
        message = "Please relax your foot"
        self.update_gui(message)
        rospy.sleep(5)
        baseline_torque = np.median(self._torque_array)
        min_torque = np.min(np.abs(self._torque_array))
        print(
            f"Collected baseline_torque={baseline_torque} and min_torque={min_torque}")
        return baseline_torque, min_torque

    def _collect_max_torque(self):
        print("Collecting max torque...")
        self._MVC_torque_array = []

        message = "Please press your foot down"
        self.update_gui(message)
        rospy.sleep(5)
        mvc1 = np.max(np.abs(self._torque_array))
        print(f"MVC1: {mvc1}")

        message = "Please relax your foot"
        self.update_gui(message)
        rospy.sleep(5)
        self._MVC_torque_array = []

        message = "Please lift your foot up"
        self.update_gui(message)
        rospy.sleep(5)
        mvc2 = np.max(np.abs(self._torque_array))
        print(f"MVC2: {mvc1}")

        message = "Please relax your foot"
        self.update_gui(message)
        rospy.sleep(5)
        self._MVC_torque_array = []

        self.window.destroy()
        self.window.mainloop()

        return np.average([mvc1, mvc2])
