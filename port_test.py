import serial
import os
from dotenv import load_dotenv

load_dotenv()

DATA_PORT = os.getenv('DATA_PORT')
BAUDRATE = 921600
MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'

try:
    ser = serial.Serial(DATA_PORT, BAUDRATE, timeout=1)
    print(f"Listening on {DATA_PORT} at {BAUDRATE} baud...\nPress Ctrl+C to stop.")

    buffer = b""
    while True:
        data = ser.read(100)
        if data:
            buffer += data
            print(f"Received {len(data)} bytes: {data.hex()}")

            # Limit buffer size
            if len(buffer) > 2048:
                buffer = buffer[-1024:]

            # Look for magic word
            if MAGIC_WORD in buffer:
                print("\n[SUCCESS] Magic word detected – port is working correctly.")
                break

except serial.SerialException as e:
    print(f"[ERROR] Serial error: {e}")
except KeyboardInterrupt:
    print("\n[INFO] Stopped by user.")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()

# import serial
# import os
# from dotenv import load_dotenv
# load_dotenv()

# DATA_PORT = os.getenv('DATA_PORT')
# BAUDRATE = 921600

# try:
#     ser = serial.Serial(DATA_PORT, BAUDRATE, timeout=1)
#     print(f"Listening on {DATA_PORT} at {BAUDRATE} baud...\nPress Ctrl+C to stop.")
    
#     while True:
#         data = ser.read(100)  # Read up to 100 bytes at a time
#         if data:
#             print(f"Received {len(data)} bytes: {data.hex()}")

# except serial.SerialException as e:
#     print(f"Serial error: {e}")
# except KeyboardInterrupt:
#     print("\nStopped by user.")
# finally:
#     if 'ser' in locals() and ser.is_open:
#         ser.close()
