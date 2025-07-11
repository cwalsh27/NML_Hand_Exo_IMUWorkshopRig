# Description: Example of reading IMU angles from a HandExo device using Python.

import time
from nml_hand_exo import HandExo

# Initialize the HandExo device with the specified serial port and baudrate.
exo = HandExo(port='COM6', baudrate=57600, verbose=False)

# Wait for the device to be ready.
time.sleep(1)

# Continuously read and print the roll, pitch, and yaw angles from the IMU.
while True:
    rpy = exo.get_imu_angles()
    print(rpy)
    time.sleep(0.1)