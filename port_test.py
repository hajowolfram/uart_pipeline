import serial
import os
from dotenv import load_dotenv
load_dotenv()

DATA_PORT = os.getenv('DATA_PORT')
PORT = "/dev/cu.usbmodemR00810384" # CHANGE
BAUDRATE = 921600

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print(f"Listening on {PORT} at {BAUDRATE} baud...\nPress Ctrl+C to stop.")
    
    while True:
        data = ser.read(100)  # Read up to 100 bytes at a time
        if data:
            print(f"Received {len(data)} bytes: {data.hex()}")

except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
