# NML_Hand_Exo

![Python](https://img.shields.io/badge/python-3.10-blue)
![License](https://img.shields.io/badge/license-MIT-green)

This repository holds tools and demonstrations for using the NML Hand Exo device. 

<!-- ![](/assets/hand-exo.jpg) -->

<p align="center">
  <img src="assets/hand-exo.jpg" width="80%"/>
</p>


Code was written and tested using Windows 11, Python 3.10.

## Setup

### Installation 
1. (Choose one): Create a virtual environment using [Anaconda](https://www.anaconda.com/products/distribution) or Python's virtualenv
   - Using Anaconda:
      ~~~
      conda create -n handexo
      conda activate handexo
      ~~~
   - ... or using Python's virtualenv:
     ~~~
     python3 -m venv .handexo
     source .handexo/bin/activate # Linux
     call .handexo/Scripts/activate # Windows
     ~~~
     
2. Clone the repository and navigate to the project directory
   ~~~
   git clone https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo.git
   cd NML_Hand_Exo
   ~~~
   
3. Install Python dependencies
    ~~~
    pip install -r requirements.txt
    ~~~
   
### Exo Firmware

The exo device uses an [openRB-150](https://emanual.robotis.com/docs/en/parts/controller/openrb-150/) microcontroller from ROBOTIS. The code to flash onto the microcontroller is located in the `firmware` directory and uploaded using an [Arduino IDE](https://www.arduino.cc/en/software/). 

The firmware includes a class NMLHandExo, which handles:

- Dynamixel initialization and setup
- Motor control by ID, name, or alias
- Joint limits and angle-to-position conversion
- Calibration and LED feedback
- Serial command parsing

To upload the firmware:

1. Open `nml_hand_exo.ino` in the Arduino IDE.
2. Select the correct board and port under Tools.
3. Upload the sketch.

## Usage

You can control the hand exoskeleton over USB or Bluetooth using simple, structured serial commands. Each command must end with a `;` delimiter. The commands are structured as follows:

```plaintext
| Command                      | Description                                                      |
|-----------------------------|------------------------------------------------------------------|
| `set_joint:<alias>:<value>` | Set position of a single joint (0–1023)                          |
| `set_joints:<alias1>:<val1>,<alias2>:<val2>,...` | Set multiple joints in one line               |
| `get_joint:<alias>`         | Query current raw position of a joint                             |
| `get_joints`                | Print all joint positions                                         |
| `set_angle:<alias>:<deg>`   | Set angle in degrees (0°–300°) for a joint                        |
| `get_angle:<alias>`         | Get current angle (degrees) of a joint                            |
| `calibrate_zero:<alias>`    | Set current joint position as zero offset                        |
| `led:<alias>:on/off`        | Toggle LED on a single joint                                     |
| `reboot:<alias>`            | Reboot a motor (useful if it becomes unresponsive)               |
```

### 🧠 Aliases Supported

- `WRIST`, `THUMB`, `INDEX`, `MIDDLE`, `RING`, `PINKY`

Values are raw position values within joint limits defined for each motor. Each command also ends with a ";\n" delimiter. You can send multiple commands in one message. Supported aliases are `THUMB`, `INDEX`, `MIDDLE`, `RING`, `PINKY`, `WRIST`

```python
import serial
import time

ser = serial.Serial('COMX', 115200)  # Replace COMX with your actual port
time.sleep(1)

# Set index and thumb finger positions
ser.write(b"set_joints:THUMB:600,INDEX:650;\n")

# Set wrist to 30 degrees
ser.write(b"set_angle:WRIST:30;\n")

# Turn off LED on thumb
ser.write(b"led:THUMB:off;\n")

ser.close()
```
## Demo

#### MindRove EMG Streaming

![](/assets/pyqtemg.gif)

A demo script is included to showcase real-time plotting of EMG signals from a connected MindRove EMG band. 
1) Connect your MindRove EMG Band to the PC (using a Wifi dongle if you want to maintain internet connection on a separate wifi network)
2) Run the demo script
   ~~~
   python demo_mindrove_realtime.py
   ~~~
## License

This project is licensed under the MIT License.
