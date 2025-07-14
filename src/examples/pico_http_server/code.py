import os
import asyncio
import time
import wifi # Comment out to avoid issues on non-WiFi boards

import socketpool
from nml_exo_webpage import WebPage
from adafruit_httpserver import Server, Request, Response, POST

from usbserialreader import USBSerialReader


def url_decode(encoded_str):
    from binascii import unhexlify
    result = ""
    i = 0
    while i < len(encoded_str):
        if encoded_str[i] == '%':
            hex_val = encoded_str[i+1:i+3]
            try:
                result += chr(int(hex_val, 16))
                i += 3
            except ValueError:
                result += '%'
                i += 1
        elif encoded_str[i] == '+':
            result += ' '
            i += 1
        else:
            result += encoded_str[i]
            i += 1
    return result



class GUIState:
    def __init__(self, n_motors=6):
        self.dark_mode = False
        self.connected = False
        self.port = ""
        self.baud = 9600
        self.operating_mode = "GESTURE_FIXED"
        self.motor_values = [0] * n_motors
        self.log_text = ""


class AsyncController(object):
    """
    This object handles serial command inputs and directs the servo positions
    according to the commands given, optionally via TCP/IP.
    """
    def __init__(self, name='ExoController',
                       ip=None,
                       port=1000,
                       rate=100,
                       use_wifi=False,
                       use_serial=False,
                       baudrate=9600,
                       use_uart=True,
                       command_delimiter=";",
                       argument_delimiter=":",
                       verbose=False):

        self.name = name
        self.ip = ip
        self.port = port
        self.rate = rate
        self.use_wifi = use_wifi
        self.use_serial = use_serial
        self.use_uart = use_uart
        self.command_delimiter = command_delimiter
        self.argument_delimiter = argument_delimiter
        self.verbose = verbose

        self.queue = []
        self.all_stop = False
        
        # Establish peripheral connection types
        self.logger("Setting up Serial support ...")
        if self.use_serial:
            self.serial = USBSerialReader(
                            use_UART=self.use_uart, 
                            baudrate=baudrate,
                            terminator='\n',
                            command_delimiter=self.command_delimiter, 
                            argument_delimiter=self.argument_delimiter, 
                            verbose=self.verbose)
            
        # WIFI and webpage 
        if self.use_wifi: 
            self.logger("Setting up wifi connection")
            self.log_text = ""
            self.n_motors = 6
            self.pool = socketpool.SocketPool(wifi.radio)
            self.server = Server(self.pool, "/static", debug=True)
            self.dark_mode = False  # default
            self.gui_state = GUIState(n_motors=self.n_motors)


        self.logger(f"{self.name} created.")
        
        # Sending first message after object intialization
        #self.serial.send("version\n");

    def logger(self, *argv, warning=False, level="debug", verbose=False):
        msg = ''.join(argv)
        prefix = "(Warning) " if warning else ""
        print("[{:.3f}][{}] {}{}".format(time.monotonic(), self.name, prefix, msg))

    async def main(self):
        if self.use_serial:
            self.logger("USB Serial Parser set up. Reading serial commands")
            asyncio.create_task(self.serial_client(30))

        if self.use_wifi:
            if not self.server:
                self.logger("Error: Failed to connect to WiFi", warning=True)
            else:
                self.setup_routes()
                try:
                    wifi.radio.hostname = "handexo"
                    if self.connect_to_wifi():                
                        self.ip = str(wifi.radio.ipv4_address)
                        self.server.start(self.ip)
                        self.logger(f"HTTP server running at http://{self.ip}")
                        #self.logger("Also accessible (if supported) via: http://handexo.local")
                    else:
                        self.logger("Failed to connect to Wi-Fi", warning=True)
                except OSError:
                    self.logger("Failed to start HTTP server", warning=True)
                    microcontroller.reset()

        asyncio.create_task(self.update(self.rate))

        if self.verbose: self.logger(f"{self.name} running!")

        while not self.all_stop:
            if self.use_wifi:
                try:
                    self.server.poll()
                except Exception as e:
                    self.logger(f"HTTP server error: {e}", warning=True)
            await asyncio.sleep(0)
            
    def connect_to_wifi(self):
        """ Makes an attempt to connect to the network given the credentials.
        
        Note: Since we are using the asyncronous server initialization, we don't need to refer to an object. We can just pass a boolean when 
        the connection is successful or not
        """
        try:
            if self.verbose: self.logger("Attempting to connect to the network...")
            wifi.radio.connect(os.getenv('CIRCUITPY_WIFI_SSID'), os.getenv('CIRCUITPY_WIFI_PASSWORD'))
            self.ip = str(wifi.radio.ipv4_address)
            if self.verbose: self.logger("Connection to local network successful")
            return True
        except:
            self.logger("Warning: Error with connecting to wifi")
            return False   
            
    async def update(self, interval=200):
        while not self.all_stop:
            if self.queue:
                msg = self.queue.pop(0)
                if self.verbose:
                    self.logger(f"Reading {msg} from list")
                await self.parse_command([msg])
            await asyncio.sleep(1 / interval)
        self.logger("Main loop exited")

    async def serial_client(self, rate):
        while not self.all_stop:
            self.serial.update()
            if self.serial._out_data:
                commands = self.serial.out_data
                for cmd in commands:
                    if self.verbose:
                        self.logger(f"Received via serial: {cmd}")
                    self.queue.append(cmd)
            #await asyncio.sleep(1 / rate)
            await asyncio.sleep(0.01)

    async def serve_client(self, reader, writer):
        addr = writer.get_extra_info('peername')
        self.logger(f"Client connected: {addr}")
        try:
            while not self.all_stop:
                data = await reader.read(100)
                if not data:
                    break
                msg = data.decode("utf-8").strip()
                if self.verbose:
                    self.logger(f"Received from network: {msg}")
                self.serial.send(msg)
        except Exception as e:
            self.logger(f"Network error: {e}", warning=True)
        finally:
            self.logger(f"Client disconnected: {addr}")
            writer.close()
            await writer.wait_closed()

    async def parse_command(self, cmd):
        # Implement command handling logic here
        if self.verbose:
            self.logger(f"Parsing: {cmd}")
        # Placeholder: echo to serial
        self.serial.send(cmd[0] + "\n")
        self.serial.update()
        
    def start(self):
        """ Makes a call to the asyncronous library to run a main routine """
        asyncio.run(self.main())  # Need to pass the async function into the run method to start

    def stop(self):
        """ Sets a flag to stop running all tasks """
        self.all_stop = True

    def setup_routes(self):
        """ 
        Setup https handling routes
        """
        @self.server.route("/")
        def base(request: Request):
            return Response(request, WebPage(self.gui_state), content_type='text/html')

        @self.server.route("/", POST)
        def handle_post(request: Request):
            try:
                form_data = request.form_data
                log_lines = []

                # --- Handle dark mode toggle ---
                self.gui_state.dark_mode = form_data.get("dark_mode") == "on"

                # --- Handle custom serial command ---
                #direct_msg = form_data.get("direct_msg", "").strip()
                direct_msg = url_decode(form_data.get("direct_msg", "").strip())
                if direct_msg:
                    log_lines.append(f"[Custom] {direct_msg}")
                    if self.use_serial:
                        self.serial.send(direct_msg + "\n")
                    self.gui_state.log_text += "\n" + "\n".join(log_lines)
                    return Response(request, WebPage(self.gui_state), content_type='text/html')

                # --- Handle Connect/Disconnect toggle ---
                if "connect" in form_data:
                    if not self.gui_state.connected:
                        self.serial.connect()
                        self.gui_state.connected = True
                    else:
                        self.serial.disconnect()
                        self.gui_state.connected = False
                    log_lines.append("Connected" if self.gui_state.connected else "Disconnected")

                # --- Update operating mode if changed ---
                selected_mode = form_data.get("mode", self.gui_state.operating_mode)
                if selected_mode != self.gui_state.operating_mode:
                    self.gui_state.operating_mode = selected_mode
                    log_lines.append(f"Mode: {selected_mode}")

                # --- Handle motor updates ---
                for i in range(len(self.gui_state.motor_values)):
                    angle_val = form_data.get(f"motor{i}") or form_data.get(f"angle{i}")
                    if angle_val is not None:
                        try:
                            val = int(angle_val)
                            if self.gui_state.motor_values[i] != val:
                                self.gui_state.motor_values[i] = val
                                log_lines.append(f"Motor {i}: {val}")
                                if self.use_serial:
                                    self.serial.send(f"gc:motor{i}:{val}\n")
                                break  # Only one motor change at a time
                        except ValueError:
                            log_lines.append(f"[Invalid] motor{i} value: {angle_val}")

                # --- Update log text if anything changed ---
                if log_lines:
                    self.gui_state.log_text += "\n" + "\n".join(log_lines)

                return Response(request, WebPage(self.gui_state), content_type='text/html')

            except Exception as e:
                self.gui_state.log_text += f"\nError: {e}"
                return Response(request, WebPage(self.gui_state), content_type='text/html')


if __name__ == "__main__":

    # Create controller
    controller = AsyncController(use_wifi=True, use_serial=True, verbose=True)
    
    # Begin server 
    controller.start()

        