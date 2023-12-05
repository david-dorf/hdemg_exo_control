# Mobile hdEMG Controller for the Technaid H3 and Fourier Intelligence X2 and M1 Exoskeleton systems

## Easy Setup [Without Exoskeleton]
- Clone this repository into a ROS workspace
- Run ```catkin_make``` in the root of the workspace
- Source the workspace with ```source devel/setup.bash```
- Launch the high-density-EMG system with ```roslaunch mobile_hdemg_exo emg_core.launch``` and select
the File option in the GUI pop-up to use a prerecorded dataset
### Usage with Exoskeletons
- Launch the Technaid H3 exoskeleton with ```roslaunch mobile_hdemg_exo h3_launch.launch```
- Launch the Fourier Intelligence M1 exoskeleton with ```roslaunch mobile_hdemg_exo m1_launch.launch```
- Launch the Fourier Intelligence X2 exoskeleton with ```roslaunch mobile_hdemg_exo x2_launch.launch```

## Overview

This software is intended to allow for high density EMG control of various exoskeletons for rehabilitation at Shirley Ryan AbilityLab. The package is designed to run on the **Jetson Orin Nano Developer Kit**, but can also work on any Ubuntu 20.04 device. The EMG package, ```mobile_hdemg_exo``` connects to and reads data from an **OTB Quattrocento** or **OTB Muovi+Pro** EMG device, and can also process prerecorded datasets. The package processes the raw EMG data with either a root-mean-squared method or a cumulative spike train convolutional neural network approach. It then calibrates the EMG data to the torque sensor data on the exoskeleton, and the EMG coefficients are used to convert the patient's EMG to accurate predicted torque commands to the exoskeleton. The exoskeleton control and read/write torque data from their respective device.
This package also provides an interface that speaks and displays instructions to the patient for them to move, in order to calibrate the EMG to their intended torque. 

If using the Jetson Orin Nano or Raspberry Pi, there is an optional latency analyzer to measure system processing time across each individual physical component, as well as each system process running on the exoskeleton and Orin Nano. The Orin Nano generates a 3.3V PWM output on GPIO pin 33 into an input on the EMG device. The processed signal is sent back over ethernet to the Orin Nano. The inverse of the difference in frequency between the PWM input and EMG output is the EMG device's latency. The delay of the EMG processing, calibration, and exoskeleton torque command is determined by using rostopic delay on their respective topics.

## Software Architecture
![latencychart](https://github.com/Technaid-S-L/technaid_h3_ankle_ros_python/assets/113081373/b294f94c-8046-41e6-9f5a-710b0d6b98c1)

## Hardware Setup
![hardware](https://github.com/Technaid-S-L/technaid_h3_ankle_ros_python/assets/113081373/f0cd0a10-c66b-4bc8-b535-b7f9276eb0a0)

## Latency Analyzer Architecture
![latency_analyzer](https://github.com/Technaid-S-L/technaid_h3_ankle_ros_python/assets/113081373/ffa85a23-80ef-410d-be35-f5984ff19d2c)

## Power Architecture
![power](https://github.com/Technaid-S-L/technaid_h3_ankle_ros_python/assets/113081373/f8f1b4db-32b2-464e-a698-3a3014793764)


## Software Setup
### Dependencies
- Ubuntu 20.04
- ROS Noetic
- Git
- Pip
- Python 3
- TensorFlow 2.13.0
- SciPy 1.7.0
- NumPy 1.22.4
- Pandas
- Pyttsx3 and espeak [Text to speech]
- Tkinter, PyQt [GUI]


### Jetson Orin Nano
- Please refer to the setup instructions here: https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit
- The Jetson Orin Nano OS image in the NVIDIA setup is currently a modified version of Ubuntu 20.04 which supports ROS Noetic

### ROS Noetic
- Please refer to the setup instructions here: http://wiki.ros.org/noetic/Installation/Ubuntu
- Install the ROS Noetic ros_control package with ```sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers```

### EMG Connections
#### Quattrocento
- Go to Network Settings and click the Settings cog next to the wired connection
- Access the IPv4 tab and enable Link-Local Only, then hit Apply
- See the OTB manual for setting up the Quattrocento hardware and connecting over Ethernet: https://www.otbioelettronica.it/en/downloads#41-quattrocento

#### Muovi+Pro
- See the OTB manual for setting up the Muovi+Pro hardware https://www.otbioelettronica.it/en/downloads#64-muovi-pro

### PCAN [Technaid H3 Only]
The Technaid H3 Exoskeleton uses PEAK CAN as its CAN interface. The PEAK CAN driver package and the PCAN Basic API must be installed on the device with the following steps:
- Download the Linux driver package from https://www.peak-system.com/Drivers.523.0.html?&L=1
- Download the Linux PCAN Basic API from https://www.peak-system.com/PCAN-Basic.239.0.html?&L=1
- Extract the two downloaded ZIP files and move the two extracted folders to /usr/include/
- In the driver directory, run the following commands in a terminal window to build the driver packages:
  - ```sudo apt-get install libpopt-dev```
  - ```sudo make clean all```
  - ```sudo make install```
  - ```sudo modprobe pcan```

### GPIO Pin Configuration [Jetson Orin Nano Only]
In order to use PWM and other additional GPIO modifications, the steps below must be taken:
- Download the NVIDIA GPIO interface library with `git clone https://github.com/NVIDIA/jetson-gpio.git`
- In the root of the cloned directory, run `sudo python3 setup.py install`
- Run the following commands to add permissions to access the GPIO user group:
  - `sudo groupadd -f -r gpio`
  - `sudo usermod -a -G gpio your_user_name` and replace your_user_name with the one in your command line interface
  - `sudo cp lib/python/Jetson/GPIO/99-gpio.rules /etc/udev/rules.d/`
  - `sudo udevadm control --reload-rules && sudo udevadm trigger`
  - `sudo /opt/nvidia/jetson-io/jetson-io.py` and change Pin 33 in the pinmux configuration to enable PWM on Pin 33. Save and reboot when prompted for the hardware changes to properly initiate.
