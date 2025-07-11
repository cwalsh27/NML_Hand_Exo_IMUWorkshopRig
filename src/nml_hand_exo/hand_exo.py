import numpy as np
import time
import serial


class HandExo(object):
    """
    Class to control the NML Hand Exoskeleton via serial communication.

    Features:
      - Enable/disable motors
      - Move motors to specific angles
      - Query status (angle, torque, current)
      - Configure velocity and acceleration
      - Retrieve device information
      - Send low-level serial commands

    """

    def __init__(self, name='NMLHandExo', port: str = None, baudrate: int = 57600, command_delimiter: str = '\n', send_delay: float = 0.01,
                 verbose: bool = False):
        """ 
        Initializes the HandExo interface.
        
        Args:
            name (str): Name of the exoskeleton instance.
            port (str): Serial port to connect to (e.g., 'COM3' or '/dev/ttyUSB0').
            baudrate (int): Baud rate for the serial connection (default is 57600).
            command_delimiter (str): Delimiter used to separate commands (default is '\n').
            send_delay (float): Delay in seconds after sending a command to allow processing (default is 0.01).
            verbose (bool): If True, enables verbose logging of commands and responses (default is False).

        """

        self.name = name
        self.port = port
        self.baudrate = baudrate
        self.command_delimiter = command_delimiter
        self.send_delay = send_delay
        self.verbose = verbose

        self.device = None
        self.connected = False
        if self.port is not None:
            self.connect(self.port, self.baudrate)

    def logger(self, *argv, warning: bool = False):
        """ 
        Robust debugging print function
        
        Args:
            *argv             : (str) Messages to log.
            warning           : (bool) If True, prints the message in yellow.

        """
        if self.verbose:
            msg = ''.join(argv)
            msg = f"[{time.monotonic():.3f}][{self.name}] {msg}"

            # If a warning, print the text in yellow
            msg = f"\033[93m{msg}\033[0m" if warning else msg
            print(msg)

    def connect(self, port: str, baudrate: int):
        """
        Establishes a serial connection to the exoskeleton.

        Args:
            port (str): Serial port to connect to.
            baudrate (int): Baud rate for the serial connection.

        """
        if not self.connected:
            try:
                self.device = serial.Serial(port, baudrate, timeout=1)
                self.connected = self.device.is_open
                if self.connected:
                    print(f"Connection established on {port} at {baudrate} baud.")
            except serial.SerialException as e:
                print(f"[ERROR] Failed to connect: {e}")
        else:
            self.logger(f"Already connected to {self.port} at {self.baudrate} baud.", warning=True)

    def send_command(self, cmd: str):
        """
        Sends a command to the exoskeleton over the serial connection.

        Args:
            cmd (str): Command to send to the exoskeleton.

        """
        if not cmd.endswith(self.command_delimiter):
            cmd += self.command_delimiter
        try:
            self.device.write(cmd.encode())
            self.logger(f"Sent: {cmd.strip()}")
            time.sleep(self.send_delay)  # Allow time for the command to be processed
        except Exception as e:
            print(f"[ERROR] Failed to send command: {e}")

    def _receive(self):
        """
        Reads a response from the exoskeleton over the serial connection.
        
        Returns:
            str: The response from the exoskeleton, or an empty string if no response.

        """
        try:
            if self.device.in_waiting:
                response = self.device.read_until(self.command_delimiter.encode()).decode().strip()
                self.logger(f"Received: {response}")
                return response
        except Exception as e:
            print(f"[ERROR] Failed to read response: {e}")
        return ""

    def enable_motor(self, motor_id: (int or str)):
        """
        Enables the torque output for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to enable.

        Returns:
            None

        """
        self.send_command(f"enable:{motor_id}")

    def disable_motor(self, motor_id: (int or str)):
        """
        Disables the torque output for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to disable.

        Returns:
            None

        """
        self.send_command(f"disable:{motor_id}")

    def enable_led(self, motor_id: (int or str)):
        """
        Enables the LED for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to enable the LED for.

        Returns:
            None

        """
        self.send_command(f"led:{motor_id}:on")

    def disable_led(self, motor_id: (int or str)):
        """
        Disables the LED for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to disable the LED for.

        Returns:
            None

        """
        self.send_command(f"led:{motor_id}:off")

    def help(self) -> str:
        """
        Sends a help command to the exoskeleton to retrieve available commands.

        Returns:
            str: A string containing the help information from the exoskeleton.

        """
        self.send_command("help")
        return self._receive()

    def home(self, motor_id: (int or str)):
        """
        Sends a home command to all motors, unless a specific motor ID is provided.

        Args:
            motor_id (int or str): ID of the motor to home, or 'all' to home all motors."

        Returns:
            None

        """
        if motor_id == 'all':
            self.send_command("home:all")
        else:
            self.send_command(f"home:{motor_id}")

    def info(self) -> dict:
        """
        Retrieves information about the exoskeleton, including version and motor details.

        Returns:
            dict: A dictionary containing version and motor information.

        """
        self.send_command("info")
        raw = self._receive()
        info = {}
        if raw:
            try:
                parts = raw.split(',')
                info['version'] = parts[0]

                n_motors = int(parts[1])
                info['n_motors'] = n_motors

                for i in range(n_motors):
                    info[f'motor_{i}'] = {
                        'name': parts[2 + i * 3],
                        'angle': float(parts[3 + i * 3]),
                        'torque': float(parts[4 + i * 3])
                    }
                return info
            except Exception as e:
                print(f"[ERROR] Failed to parse info: {e}")

    def get_baudrate(self) -> int:
        """
        Retrieves the current baud rate of the serial connection.

        Returns:
            int: The current baud rate.

        """
        if self.device and self.device.is_open:
            return self.device.baudrate
        else:
            print("[ERROR] Serial device is not connected.")
            return None

    def get_motor_velocity(self, motor_id: (int or str)) -> float:
        """
        Retrieves the current velocity of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current velocity of the motor in degrees per second.

        """
        self.send_command(f"get_vel:{motor_id}")
        response = self._receive()
        try:
            return float(response.split(':')[-1])
        except (ValueError, IndexError):
            print(f"[ERROR] Invalid response for motor {motor_id}: {response}")
            return 0.0

    def set_motor_velocity(self, motor_id: (int or str), velocity: float):
        """
        Sets the velocity for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the velocity for.
            velocity (float): Desired velocity in degrees per second.

        Returns:
            None

        """
        self.send_command(f"set_vel:{motor_id}:{velocity}")

    def get_motor_acceleration(self, motor_id: (int or str)) -> float:
        """
        Retrieves the current acceleration of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current acceleration of the motor in degrees per second squared.

        """
        self.send_command(f"get_accel:{motor_id}")
        response = self._receive()
        try:
            return float(response.split(':')[-1])
        except (ValueError, IndexError):
            print(f"[ERROR] Invalid response for motor {motor_id}: {response}")
            return 0.0

    def set_motor_acceleration(self, motor_id: (int or str), acceleration: float):
        """
        Sets the acceleration for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the acceleration for.
            acceleration (float): Desired acceleration in degrees per second squared.

        Returns:
            None

        """
        self.send_command(f"set_accel:{motor_id}:{acceleration}")

    def get_motor_angle(self, motor_id: (int or str)) -> float:
        """
        Retrieves the current relative angle of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current angle of the motor in degrees.

        """
        self.send_command(f"get_angle:{motor_id}")
        response = self._receive()
        try:
            return float(response.split(':')[-1])
        except (ValueError, IndexError):
            print(f"[ERROR] Invalid response for motor {motor_id}: {response}")
            return 0.0

    def set_motor_angle(self, motor_id: (int or str), angle: float):
        """
        Sets the angle for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the angle for.
            angle (float): Desired angle in degrees.

        Returns:
            None

        """
        if isinstance(motor_id, str):
            cmd = f"set_angle:{motor_id}:{angle}"
        else:
            cmd = f"set_angle:{int(motor_id)}:{angle}"
        self.send_command(cmd)

    def get_absolute_motor_angle(self, motor_id: (int or str)) -> float:
        """
        Retrieves the absolute angle of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Absolute angle of the motor in degrees.

        """
        self.send_command(f"get_absangle:{motor_id}")
        response = self._receive()
        try:
            return float(response.split(':')[-1])
        except (ValueError, IndexError):
            print(f"[ERROR] Invalid response for motor {motor_id}: {response}")
            return 0.0

    def set_absolute_motor_angle(self, motor_id: (int or str), angle: float):
        """
        Sets the absolute angle for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the absolute angle for.
            angle (float): Desired absolute angle in degrees.

        Returns:
            None

        """
        if isinstance(motor_id, str):
            cmd = f"set_absangle:{motor_id}:{angle}"
        else:
            cmd = f"set_absangle:{int(motor_id)}:{angle}"
        self.send_command(cmd)

    def get_home(self, motor_id: (int or str)) -> float:
        """
        Retrieves the home angle of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Home angle of the motor in degrees.

        """
        self.send_command(f"get_home:{motor_id}")
        response = self._receive()
        try:
            return float(response.split(':')[-1])
        except (ValueError, IndexError):
            print(f"[ERROR] Invalid response for motor {motor_id}: {response}")
            return 0.0

    def set_home(self, motor_id: (int or str), home_angle: float):
        """
        Sets the home angle for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the home angle for.
            home_angle (float): Desired home angle in degrees.

        Returns:
            None

        """
        self.send_command(f"set_home:{motor_id}:{home_angle}")

    def get_motor_torque(self, motor_id: (int or str)) -> float:
        """
        Retrieves the current torque of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current torque of the motor in Newton-meters.

        """
        self.send_command(f"get_torque:{motor_id}")
        response = self._receive()
        try:
            return float(response.split(':')[-1])
        except (ValueError, IndexError):
            print(f"[ERROR] Invalid response for motor {motor_id}: {response}")
            return 0.0

    def get_motor_current(self, motor_id: (int or str)) -> float:
        """
        Retrieves the current draw of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current draw of the motor in Amperes.

        """
        self.send_command(f"get_current:{motor_id}")
        response = self._receive()
        try:
            return float(response.split(':')[-1])
        except (ValueError, IndexError):
            print(f"[ERROR] Invalid response for motor {motor_id}: {response}")
            return 0.0

    def get_motor_current_limit(self, motor_id: (int or str)) -> float:
        """
        Retrieves the current limit of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current limit of the motor in Amperes.

        """
        self.send_command(f"get_current_lim:{motor_id}")
        response = self._receive()
        try:
            return float(response.split(':')[-1])
        except (ValueError, IndexError):
            print(f"[ERROR] Invalid response for motor {motor_id}: {response}")
            return 0.0

    def set_current_limit(self, motor_id: (int or str), current_limit: float):
        """
        Sets the current limit for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the current limit for.
            current_limit (float): Desired current limit in Amperes.

        Returns:
            None

        """
        self.send_command(f"set_current_lim:{motor_id}:{current_limit}")

    def get_motor_status(self, motor_id: (int or str)) -> dict:
        """
        Retrieves the status of the specified motor, including angle, torque, current, velocity, and acceleration.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            dict: A dictionary containing the motor's status.

        """
        self.send_command(f"get_status:{motor_id}")
        response = self._receive()
        status = {}
        if response:
            try:
                parts = response.split(',')
                status['angle'] = float(parts[0].split(':')[-1])
                status['torque'] = float(parts[1].split(':')[-1])
                status['current'] = float(parts[2].split(':')[-1])
                status['velocity'] = float(parts[3].split(':')[-1])
                status['acceleration'] = float(parts[4].split(':')[-1])
                return status
            except (ValueError, IndexError):
                print(f"[ERROR] Invalid response for motor {motor_id}: {response}")
        return {}

    def get_motor_limits(self, motor_id: (int or str)) -> tuple:
        """
        Retrieves the limits for the specified motor, including minimum and maximum angles.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            tuple: A tuple containing the minimum and maximum angles of the motor.

        """
        self.send_command(f"get_motor_limits:{motor_id}")
        response = self._receive() # Should get "Motor {id} limits:[{val},{val}]"
        limits = None
        if response:
            try:
                parts = response.split(':')
                if len(parts) == 3 and parts[0].startswith("Motor") and parts[1].strip() == "limits":
                    limits = tuple(map(float, parts[2].strip('[]').split(',')))
                else:
                    print(f"[ERROR] Invalid response for motor {motor_id}: {response}")
            except (ValueError, IndexError):
                print(f"[ERROR] Failed to parse limits for motor {motor_id}: {response}")
        return limits if limits else (None, None)

    def set_motor_upper_limit(self, motor_id: (int or str), upper_limit: float):
        """
        Sets the upper limit for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the upper limit for.
            upper_limit (float): Desired upper limit in degrees.

        Returns:
            None

        """
        self.send_command(f"set_upper_limit:{motor_id}:{upper_limit}")

    def set_motor_lower_limit(self, motor_id: (int or str), lower_limit: float):
        """
        Sets the lower limit for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the lower limit for.
            lower_limit (float): Desired lower limit in degrees.

        Returns:
            None

        """
        self.send_command(f"set_lower_limit:{motor_id}:{lower_limit}")

    def set_motor_limits(self, motor_id: (int or str), lower_limit: float, upper_limit: float):
        """
        Sets both the lower and upper limits for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the limits for.
            lower_limit (float): Desired lower limit in degrees.
            upper_limit (float): Desired upper limit in degrees.

        Returns:
            None

        """
        self.send_command(f"set_motor_limits:{motor_id}:{lower_limit}:{upper_limit}")

    def reboot_motor(self, motor_id: (int or str)):
        """
        Reboots the specified motor.

        Args:
            motor_id (int or str): ID of the motor to reboot.

        Returns:
            None

        """
        if motor_id == 'all':
            self.send_command("reboot:all")
        else:
            self.send_command(f"reboot:{motor_id}")

    def get_motor_mode(self) -> str:
        """
        Retrieves the current control mode of the specified motor.

        Returns:
            str: Current mode of the motor (e.g., "position", "velocity", "current_position").

        """
        self.send_command(f"get_motor_mode")
        response = self._receive()
        if response:
            try:
                return response.split(':')[-1].strip()
            except IndexError:
                print(f"[ERROR] Invalid response")
        return ""

    def set_motor_mode(self, motor_id: (int or str), mode: str):
        """
        Sets the control mode for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the mode for.
            mode (str): Desired control mode (e.g., "position", "velocity", "current_position").

        Returns:
            None

        """
        self.send_command(f"set_motor_mode:{motor_id}:{mode}")

    def get_exo_mode(self) -> str:
        """
        Retrieves the current operating mode of the exoskeleton.

        Returns:
            str: Current mode of the exoskeleton (e.g., "manual", "autonomous").

        """
        self.send_command("get_exo_mode")
        response = self._receive()
        if response:
            try:
                return response.split(':')[-1].strip()
            except IndexError:
                print(f"[ERROR] Invalid response")
        return ""

    def set_exo_mode(self, mode: str):
        """
        Sets the operating mode for the exoskeleton.

        Args:
            mode (str): Desired operating mode (e.g., "manual", "autonomous").

        Returns:
            None

        """
        self.send_command(f"set_exo_mode:{mode}")

    def get_gesture(self) -> str:
        """
        Retrieves the current gesture recognized by the exoskeleton.

        Returns:
            str: Current gesture (e.g., "open", "close", "pinch").

        """
        self.send_command("get_gesture")
        response = self._receive()
        if response:
            try:
                return response.split(':')[-1].strip()
            except IndexError:
                print(f"[ERROR] Invalid response")
        return ""

    def set_gesture(self, gesture: str, state: str = "default"):
        """
        Sets the gesture for the exoskeleton.

        Args:
            gesture (str): Desired gesture (e.g., "open", "close", "pinch").

        Returns:
            None

        """
        self.send_command(f"set_gesture:{gesture}:{state}")

    def get_gesture_list(self) -> list:
        """
        Retrieves the list of available gestures for the exoskeleton.

        Returns:
            list: A list of available gestures.

        """
        self.send_command("gesture_list")
        response = self._receive()
        if response:
            try:
                return response.split(':')[-1].strip().split(',')
            except IndexError:
                print(f"[ERROR] Invalid response")
        return []

    def cycle_gesture(self):
        """
        Cycles through the available gestures for the exoskeleton.

        Returns:
            None

        """
        self.send_command("cycle_gesture")

    def cycle_gesture_state(self):
        """
        Cycles through the states of the current gesture for the exoskeleton.

        Returns:
            None

        """
        self.send_command("cycle_gesture_state")

    def close(self):
        """
        Closes the serial connection to the exoskeleton.

        Returns:
            None

        """
        if self.device and self.device.is_open:
            self.device.close()
            self.logger("Serial connection closed.")

    def get_imu_data(self) -> dict:
        """
        Retrieves the IMU data from the exoskeleton.

        Receives a serial message with contents, as an example:
            "Temp: 20.28 C; Accel: [-0.46, -0.42, 9.85]; Gyro: [-0.00, 0.01, -0.00];"

        Returns:
            dict: A dictionary containing IMU data (e.g., accelerometer, gyroscope, magnetometer).

        """
        self.send_command("get_imu")
        response = self._receive()
        imu_data = {}
        if response:
            try:
                parts = response.split(';')
                for part in parts:
                    part = part.strip()
                    if part.startswith("Temp:"):
                        imu_data['temperature'] = float(part.split(':')[-1].strip().replace('C', ''))
                    elif part.startswith("Accel:"):
                        accel_str = part.split(':')[-1].strip().strip('[]')
                        imu_data['acceleration'] = list(map(float, accel_str.split(',')))
                    elif part.startswith("Gyro:"):
                        gyro_str = part.split(':')[-1].strip().strip('[]')
                        imu_data['gyroscope'] = list(map(float, gyro_str.split(',')))
                    # Add more parts as needed (e.g., magnetometer)

                return imu_data
            except (ValueError, IndexError):
                print(f"[ERROR] Invalid IMU data response: {response}")
        return {}

    def get_imu_angles(self, degrees=True, raw=False) -> dict:
        """
        Computes the orientation of the exo device based on the imu data received

        Args:
            degrees (bool): If True, returns roll, pitch, and yaw in degrees. If False, returns in radians.
            raw (bool): If True, returns raw IMU data instead of computed orientation.

        Returns:
            dict: A dictionary containing orientation data (e.g., roll, pitch, yaw).
        """
        data = self.get_imu_data()
        if data:
            if raw:
                return data

            # compute the roll, pitch, and yaw from the accelerometer and gyroscope data
            accel = data.get('acceleration', [0, 0, 0])
            gyro = data.get('gyroscope', [0, 0, 0])
            roll = np.atan2(accel[1], accel[2])
            pitch = np.atan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
            yaw = np.atan2(gyro[1], gyro[0])

            if degrees:
                roll = np.degrees(roll)
                pitch = np.degrees(pitch)
                yaw = np.degrees(yaw)

            return {
                'roll': float(roll),
                'pitch': float(pitch),
                'yaw': float(yaw)
            }