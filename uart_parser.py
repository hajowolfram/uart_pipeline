import serial

def basic_uart_read(port='COM5', baudrate=921600):
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1)
        print(f"Listening on {port} at {baudrate} baud...\nPress Ctrl+C to stop.\n")
        
        while True:
            data = ser.read(100)  # read up to 100 bytes
            if data:
                print(f"Received {len(data)} bytes: {data.hex()}")
    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

"""
PACKET STRUCTURE OVERVIEW
-----
MAGIC_WORD = 02 01 04 03 06 05 08 07 -> 
HEADER
TLV_1
TLV_2
  .
  .
TLV_N
-----

MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'

"""

if __name__ == "__main__":
    basic_uart_read()
