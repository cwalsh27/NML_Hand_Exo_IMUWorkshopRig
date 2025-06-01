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
            name  (str): Name of the exoskeleton instance.
            port  (str): Serial port to connect to (e.g., 'COM3' or '/dev/ttyUSB0').
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

    def reboot_motor(self, motor_id: (int or str)):
        """
        Reboots the specified motor.

        Args:
            motor_id (int or str): ID of the motor to reboot.

        Returns:
            None

        """
        self.send_command(f"reboot:{motor_id}")

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

    def close(self):
        """
        Closes the serial connection to the exoskeleton.

        Returns:
            None

        """
        if self.device and self.device.is_open:
            self.device.close()
            self.logger("Serial connection closed.")