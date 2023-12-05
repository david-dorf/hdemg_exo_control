import tkinter as tk
from tkinter import ttk
import rospy

SAMPLE_FREQUENCY = rospy.get_param("/sampling_frequency", int)
EXO_CONNECTED = rospy.get_param("/connected_to_exo", bool)


class StartupGUINode:
    """
    A ROS node for the startup GUI.

    Attributes:
        r: A ROS Rate object.
        root: A Tkinter root object.
        device_var: A Tkinter StringVar for the device dropdown.
        muscles_var: A Tkinter StringVar for the muscles dropdown.
        remove_channels_var: A Tkinter StringVar for the channels to remove entry.

    Methods:
        create_widgets: Creates the Tkinter widgets.
        start_process: Starts the EMG processing node.
    """

    def __init__(self, root):
        rospy.init_node('emg_processor_node')
        self.r = rospy.Rate(SAMPLE_FREQUENCY)
        self.root = root
        self.root.title("SRAL hdEMG Exoskeleton")
        self.device_var = tk.StringVar()
        self.muscles_var = tk.StringVar()
        self.device_var.set("File")
        self.muscles_var.set("1")
        self.root.geometry("420x200")
        self.create_widgets()
        if EXO_CONNECTED:
            from mobile_hdemg_exo.calibration.trial import Trial, TrialDirection, TrajectoryShape
            from mobile_hdemg_exo.calibration.trial_runner import TrialRunner
            self.trial = Trial(0, TrialDirection.NoDirection,
                               TrajectoryShape.Flat, 0, 25)

    def create_widgets(self):
        device_label = ttk.Label(self.root, text="Device:")
        device_label.grid(row=0, column=0, padx=10, pady=5, sticky="w")
        device_choices = ["Muovi+Pro", "Quattrocento", "File"]
        self.device_dropdown = ttk.Combobox(
            self.root, values=device_choices, textvariable=self.device_var)
        self.device_dropdown.grid(row=0, column=1, padx=10, pady=5, sticky="w")

        muscles_label = ttk.Label(self.root, text="Number of probes:")
        muscles_label.grid(row=1, column=0, padx=10, pady=5, sticky="w")
        muscles_choices = ["1", "2", "3", "4"]
        self.muscles_dropdown = ttk.Combobox(
            self.root, values=muscles_choices, textvariable=self.muscles_var)
        self.muscles_dropdown.grid(
            row=1, column=1, padx=10, pady=5, sticky="w")

        remove_channels_label = ttk.Label(
            self.root, text="Remove channels [1,2,...]:")
        remove_channels_label.grid(
            row=2, column=0, padx=10, pady=5, sticky="w")
        self.remove_channels_var = tk.StringVar()
        remove_channels_entry = ttk.Entry(
            self.root, textvariable=self.remove_channels_var)
        remove_channels_entry.grid(
            row=2, column=1, padx=10, pady=5, sticky="w")

        start_button = ttk.Button(
            self.root, text="Start EMG", command=self.start_process)
        start_button.grid(row=3, columnspan=2, pady=10)

        if EXO_CONNECTED:
            run_trial_button = ttk.Button(
                self.root, text="Run Trial", command=self.run_trial)
            run_trial_button.grid(row=4, columnspan=2, pady=10)

    def start_process(self):
        selected_device = self.device_var.get()
        selected_muscles = int(self.muscles_var.get())
        channels_to_remove = self.remove_channels_var.get()

        rospy.set_param("/device", selected_device)
        rospy.set_param("/muscle_count", selected_muscles)
        rospy.set_param("/channels_to_remove", channels_to_remove)

        print("Device:", selected_device)
        print("Muscles:", selected_muscles)
        print("Removed Channels:", channels_to_remove)

        rospy.set_param("/startup_gui_completed", True)

    def run_trial(self):
        TrialRunner(self.trial).collect_trial_data()
        rospy.set_param("calibrated", True)


if __name__ == "__main__":
    root = tk.Tk()
    app = StartupGUINode(root)
    while not rospy.is_shutdown():
        root.update_idletasks()
        root.update()
        app.r.sleep()
