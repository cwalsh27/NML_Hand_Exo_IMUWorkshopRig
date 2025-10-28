
import numpy as np
import pylsl
from pylsl import StreamInlet
import time
import csv
import threading
import tkinter as tk
import math

from nml_hand_exo.interface import HandExo, SerialComm



############### Parameters ###############          TODO: add functions to hand_exo so that we can pivot to roll or pitch if need be 
PORT = 'COM5'
BAUD_RATE = 57600
motorID = 0

stream_names = ['FLX', 'EXT']
sampling_rate = 2000  # Hz


############### Globals ###############
stop_event = threading.Event()
filename_timestamp = int(time.time())
inlets = []
channel_labels = []
current_trigger_value = 0  

shared_imu_data = [0, 0, 0]
imu_lock = threading.Lock()

imu_update_enabled = threading.Event()
imu_update_enabled.set()


############### Connect to LSL Streams ###############
print("Resolving LSL streams...")
streams = pylsl.resolve_streams()

for name in stream_names:
    found = False
    for stream in streams:
        if stream.name() == name:
            print(f"Connected to LSL stream: {name}")
            inlet = StreamInlet(stream, max_buflen=60)
            inlets.append((name, inlet))
            found = True
            break
    if not found:
        print(f"Warning: LSL stream '{name}' not found.")

if len(inlets) == 0:
    raise RuntimeError("No required LSL streams found. Check stream names or network.")

############### Connect to Exo on COM Port ###############
comm = SerialComm(port=PORT, baudrate=BAUD_RATE)
exo = HandExo(comm, verbose=False)
exo.connect()
print('exo connected')

############### Extract Channel Labels ###############
def get_channel_labels(info, n_channels):
    labels = []
    try:
        chns = info.desc().child("channels").child("channel")
        for _ in range(n_channels):
            label = chns.child_value("label")
            labels.append(label if label else f"ch{_}")
            chns = chns.next_sibling()
    except:
        labels = [f"ch{i}" for i in range(n_channels)]
    return labels

for name, inlet in inlets:
    info = inlet.info()
    n_channels = info.channel_count()
    channel_labels.append((name, get_channel_labels(info, n_channels)))

print("Channel labels:")
for name, labels in channel_labels:
    print(f"{name}: {labels}")


############### Trigger Function ###############
def send_analog_trigger(value):
    global current_trigger_value
    current_trigger_value = value
    # print(f"Trigger = {value}")

############### IMU Management Thread ############### 
def imu_manager():
    global shared_imu_data
    while not stop_event.is_set():
        if imu_update_enabled.is_set():
            with imu_lock:
                dir = exo.get_imu_angles()
                if dir is not None:
                    shared_imu_data = dir
        time.sleep(0.01)

def get_latest_imu_angle():
        with imu_lock:
            # print(shared_imu_data)
            return shared_imu_data

############### Data Recording Thread ###############
def record_data(inlet, name):
    filename = f"rom_lsl_record_{name}_{filename_timestamp}.csv"
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)

        while not stop_event.is_set():
            sample, timestamp = inlet.pull_sample()
            # print(sample)
            dir = get_latest_imu_angle()
            writer.writerow([timestamp] + sample + dir + [current_trigger_value])

        print(f"{name} recording stopped. Data saved to {filename}")



def control_structure():
    global rest_wrist_angle, extended_wrist_angle, flexed_wrist_angle

    with imu_lock:
        exo.disable_motor(motor_id=motorID)
    '''Calibration Stage'''
    
    wait = input("Begin with wrist at neutral position. Hit enter when ready.")
    send_analog_trigger(1)
    _, rest_timestamp = inlet.pull_sample()
    trigger_timestamps.append((f"rest", rest_timestamp))
    with imu_lock:
        rest_motor_angle = exo.get_motor_angle(motor_id=motorID)
    print("here")
    rest_wrist_angle = get_latest_imu_angle()
    print("rest motor: ", rest_motor_angle, "zero wrist", rest_wrist_angle)

    wait = input("Move patients wrist to be as flexed as possible, whether the limit is physiological or mechanical. This will be the point to which the exo flexes the wrist.\n Hit any key to record the wrist/motor angle.")
    send_analog_trigger(2)
    _, flex_timestamp = inlet.pull_sample()
    trigger_timestamps.append((f"flex", flex_timestamp))
    with imu_lock:
        flexed_motor_angle = exo.get_motor_angle(motor_id=motorID)
    flexed_wrist_angle = get_latest_imu_angle()
    print("flexed motor:", flexed_motor_angle, "flexed wrist:", flexed_wrist_angle)

    wait = input("Move patients wrist to be as extended as possible, whether the limit is physiological or mechanical. This will be the point to which the exo extends the wrist.\n Hit any key to record the wrist/motor angle.")
    send_analog_trigger(3)
    _, extend_timestamp = inlet.pull_sample()
    trigger_timestamps.append((f"extend", extend_timestamp))
    with imu_lock: 
        extended_motor_angle = exo.get_motor_angle(motor_id=motorID)
    extended_wrist_angle = get_latest_imu_angle()
    print("extended motor:", extended_motor_angle, "extended wrist:", extended_wrist_angle)
    
    
    
    with open(f"rom_trigger_timestamps_{filename_timestamp}.txt", 'w') as timeFile:
        timeFile.write(" ".join(map(str, trigger_timestamps)))


    with open(f"rom_results_{filename_timestamp}.txt", 'w') as resultFile:
        resultFile.write(f"rest motor: {rest_motor_angle}, rest wrist: {rest_wrist_angle}\n")
        resultFile.write(f"flexed motor: {flexed_motor_angle}, flexed wrist: {flexed_wrist_angle}\n")
        resultFile.write(f"extended motor: {extended_motor_angle}, extended wrist: {extended_wrist_angle}\n")
       

    stop_event.set()

############### Main Execution ###############
if __name__ == "__main__":

    trigger_timestamps = []

    threads = []
    for name, inlet in inlets:
        t = threading.Thread(target=record_data, args=(inlet, name))
        t.start()
        threads.append(t)


    t_imu = threading.Thread(target=imu_manager)
    t_imu.start()
    threads.append(t_imu)

    t_control = threading.Thread(target=control_structure)
    t_control.start()
    threads.append(t_control)

    for t in threads:
        t.join()

    print("Experiment completed.")
