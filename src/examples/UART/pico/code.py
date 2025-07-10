import time
import board
import busio

# Initialize UART using Pico GP16 (TX, not used) and GP17 (RX)
uart = busio.UART(tx=board.GP4, rx=board.GP5, baudrate=9600, timeout=0.1)

print("UART Echo is ready.")
while True:
    data = uart.read(32)  # Try reading up to 32 bytes
    if data:
        try:
            decoded = data.decode("utf-8").strip()
            print("Received:", decoded)
            uart.write(data)  # Echo the exact bytes back
        except UnicodeError:
            print("Received non-UTF-8 data:", data)
            uart.write(data)
            
    time.sleep(0.01)  # Small delay to prevent CPU overuse
