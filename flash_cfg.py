import serial
import time
import sys
import os
from dotenv import load_dotenv

def flash_cfg_file(serial_port, baud_rate, cfg_file_path):
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"Opened serial port {serial_port} at {baud_rate} baud.")
    except Exception as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)

    try:
        with open(cfg_file_path, 'r') as cfg_file:
            cfg_lines = cfg_file.readlines()
            print(f"Successfully read config file: {cfg_file_path}")
    except Exception as e:
        print(f"Error reading config file: {e}")
        ser.close()
        sys.exit(1)

    try:
        print("Sending config lines...")
        time.sleep(2) 

        for line in cfg_lines:
            line = line.strip()
            if not line or line.startswith('%'):
                continue

            ser.write((line + '\n').encode('utf-8'))
            ser.flush()
            print(f"Sent: {line}")
            time.sleep(0.05)

        print("Config successfully sent.")
        
        time.sleep(1)
        while ser.in_waiting > 0:
            print("Response:", ser.readline().decode().strip())

    except Exception as e:
        print(f"Error during config transmission: {e}")
    finally:
        ser.close()
        print(f"Closed serial port {serial_port}.")

if __name__ == '__main__':
    load_dotenv()
    COM_PORT = os.getenv('COM_PORT')
    baud_rate = 115200    
    CFG_FILE_PATH = os.getenv('CFG_FILE_PATH')
    flash_cfg_file(COM_PORT, baud_rate, CFG_FILE_PATH)

