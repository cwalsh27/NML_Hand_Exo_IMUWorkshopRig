from nml_hand_exo.interface import FakeHandExo, SerialComm, LSLClient
from nml_hand_exo.control import RMSGestureTrigger

if __name__ == "__main__":

    # Serial port config
    port = "COM6"
    baudrate = 115200

    comm = SerialComm(port=port, baudrate=baudrate)
    exo = FakeHandExo(comm, verbose=True)
    exo.connect()

    lsl = LSLClient(stream_type="EMG")
    trigger = RMSGestureTrigger(exo=exo, client=lsl, threshold_percent=0.10)
    trigger.client.start_streaming()

    trigger.calibrate_rest_state()

    trigger.start()

