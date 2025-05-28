
import time
import board
import busio
import asyncio
import adafruit_mpu6050
import displayio
import terminalio
import microcontroller
from adafruit_display_text import label
import adafruit_displayio_ssd1306
from usbserialreader import USBSerialReader

__author__ = "Jonathan Shulgach"
__version__ = "0.0.1"

# Release any previously initialized displays
displayio.release_displays()

# Set the cpu frequency to 200MHz for overclocking
microcontroller.cpu.frequency = 200000000  # 200MHz

class MPUController:
    """ Object for collecting data from MPU6050
    """
    def __init__(self, name="MPUController", verbose=False):
        self.name = name
        self.verbose = verbose
        self.connected = False
        self.recording = False
        self.display_task = None
        self.stream_task = None

        self.all_stop = False
        self.ax = self.ay = self.az = 0.0
        self.gx = self.gy = self.gz = 0.0
        self.gx_offset = self.gy_offset = self.gz_offset = 0.0
        self.ax_offset = self.ay_offset = self.az_offset = 0.0

        # Init I2C
        self.i2c = busio.I2C(board.GP1, board.GP0)

        # Init OLED Display
        display_bus = displayio.I2CDisplay(self.i2c, device_address=0x3C)
        self.display = adafruit_displayio_ssd1306.SSD1306(display_bus, width=128, height=32)
        self.splash = displayio.Group()
        self.display.root_group = self.splash

        self.mpu = adafruit_mpu6050.MPU6050(self.i2c)

        self.label = label.Label(terminalio.FONT, text="Initializing...", color=0xFFFFFF)
        self.label.anchor_point = (0.5, 0.5)
        self.label.anchored_position = (64, 16)
        self.label.scale = 1
        self.splash.append(self.label)

        # USB serial
        self.serial = USBSerialReader(use_UART=False, baudrate=115200, verbose=self.verbose)

    def calibrate_mpu(self, samples=100):
        self.logger("Calibrating Gyro... hold still")
        gx_sum = gy_sum = gz_sum = 0.0
        ax_sum = ay_sum = az_sum = 0.0
        for _ in range(samples):
            gx, gy, gz = self.mpu.gyro
            ax, ay, az = self.mpu.acceleration
            gx_sum += gx
            gy_sum += gy
            gz_sum += gz
            ax_sum += ax
            ay_sum += ay
            az_sum += az
            time.sleep(0.01)
        self.gx_offset = gx_sum / samples
        self.gy_offset = gy_sum / samples
        self.gz_offset = gz_sum / samples
        self.ax_offset = ax_sum / samples
        self.ay_offset = ay_sum / samples
        self.az_offset = az_sum / samples
        if self.verbose:
            self.logger(f"Offsets - gx: {self.gx_offset:.4f}, gy: {self.gy_offset:.4f}, gz: {self.gz_offset:.4f}")
            self.logger(f"Offsets - ax: {self.ax_offset:.4f}, ay: {self.ay_offset:.4f}, az: {self.az_offset:.4f}")
        self.update_oled("Idle")

    async def sample_mpu(self):
        """ Continuously sample raw IMU values. Fastest time is 100Hz """
        #t_prev = time.monotonic()
        while True:
            ax, ay, az = self.mpu.acceleration
            gx, gy, gz = self.mpu.gyro

            self.ax, self.ay, self.az = ax, ay, az
            self.gx, self.gy, self.gz = gx, gy, gz
            self.gx -= self.gx_offset
            self.gz -= self.gy_offset
            self.gx -= self.gz_offset
            self.ax -= self.ax_offset
            self.ay -= self.ay_offset
            self.az -= self.az_offset


            #t_elapsed = time.monotonic() - t_prev
            #t_prev = time.monotonic()
            #print(f"{1/t_elapsed}Hz")
            await asyncio.sleep(0)  # Sample as fast as possible

    def update_oled(self, msg):
        """ Update OLED with a message """
        self.label.text = msg
        #self.display.show(self.splash)
            
    def logger(self, *argv, warning=False):
        """ Robust printing function """
        msg = ''.join(argv)
        if warning: msg = '(Warning) ' + msg
        print("[{:.3f}][{}] {}".format(time.monotonic(), self.name, msg))
        
    def start(self):
        """ Makes a call to the asyncronous library to run a main routine """
        asyncio.run(self.main())  # Need to pass the async function into the run method to start

    def stop(self):
        """ Sets a flag to stop running all tasks """
        self.all_stop = True
        
    async def main(self):
        """ Start main tasks and coroutines in a single main function """
        self.calibrate_mpu()
        asyncio.create_task(self.sample_mpu())
        asyncio.create_task(self.serial_client())
        while not self.all_stop:
            await asyncio.sleep(0) # Calling #async with sleep for 0 seconds allows coroutines to run
            
    async def serial_client(self, interval=100):
        """ Read serial commands and add them to the command queue """
        while self.all_stop != True:
            self.serial.update() # built-in function to continuously poll for new data
            
            # If connection was lost
            if not self.serial.connected and self.connected:
                self.connected = False
                
            # A new connected was discovered
            elif self.serial.connected and not self.connected:
                self.connected = True

            # If data is available in the '_out_data' buffer, parse it
            if self.serial._out_data:
                #self.logger(f"Data: {self.serial._out_data}")
                data = self.serial.out_data
                self.parse_command(data)

            await asyncio.sleep(1 / int(interval))

    def parse_command(self, data):
        """ Handle the characters received from the serial port """
        for command in data:
            if not command:
                continue
            cmd = command[0].lower()
            if cmd == 's':
                self.update_oled("streaming")
                self.recording = True
                #self.display_task = asyncio.create_task(self.display_mpu_data_oled())
                self.stream_task = asyncio.create_task(self.stream_mpu_data())
                if self.verbose:
                    self.logger("Starting streaming...")
            elif cmd == 'q':
                self.update_oled("stopped")
                self.recording = False
                if self.verbose:
                    self.logger("Stopping recording...")
                if hasattr(self, 'stream_task'):
                    self.stream_task.cancel()
                    self.logger("Cancelled stream task")
            else:
                self.logger(f"Unknown command: '{command}'")
                self.update_oled("unknown cmd")

    async def display_mpu_data_oled(self, rate=2):
        """ Update OLED every 0.5s """
        while True:
            self.label.text = f"A:{self.ax:.1f},{self.ay:.1f}"
            await asyncio.sleep(1/rate)

    async def stream_mpu_data(self, rate=100):
        """ Send raw values over USB serial every 0.1s """
        while self.all_stop != True:
            if self.connected:
                msg = f"AX:{self.ax:.2f},AY:{self.ay:.2f},AZ:{self.az:.2f},GX:{self.gx:.2f},GY:{self.gy:.2f},GZ:{self.gz:.2f}"
                self.logger(f"{msg}")

            await asyncio.sleep(1/rate)
