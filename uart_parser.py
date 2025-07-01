import serial
import struct

MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'

def find_magic_word(buffer: bytes) -> int:
    """Find the index of the magic word in the buffer."""
    return buffer.find(MAGIC_WORD)

def parse_header(buffer: bytes) -> dict:
    """Parse mmWave UART header (after magic word)."""
    if len(buffer) < 32:
        return None

    fields = struct.unpack('<8I', buffer[:32])
    return {
        'version': fields[0],
        'totalPacketLen': fields[1],
        'platform': fields[2],
        'frameNumber': fields[3],
        'timeCpuCycles': fields[4],
        'numDetectedObj': fields[5],
        'numTLVs': fields[6],
        'subFrameNumber': fields[7],
    }

def parse_tracked_objects(payload: bytes):
    if len(payload) < 4:
        print("Payload too short for tracked objects.")
        return

    num_tracks = struct.unpack('<H', payload[:2])[0]
    print(f"Number of tracked objects: {num_tracks}")

    offset = 4  # 2 bytes for count, 2 for padding
    for i in range(num_tracks):
        if offset + 28 > len(payload):
            print("Incomplete object record, skipping.")
            break

        tid, posX, posY, posZ, velX, velY, velZ = struct.unpack('<Ifffffff', payload[offset:offset + 28])
        print(f"Track {tid}: pos=({posX:.2f}, {posY:.2f}, {posZ:.2f}), vel=({velX:.2f}, {velY:.2f}, {velZ:.2f})")
        offset += 28

def parse_tlvs(buffer: bytes, num_tlvs: int):
    offset = 0
    for i in range(num_tlvs):
        if offset + 8 > len(buffer):
            print("Incomplete TLV header.")
            break

        tlv_type, tlv_length = struct.unpack('<II', buffer[offset:offset+8])
        payload = buffer[offset+8:offset+tlv_length]

        if tlv_type == 7:
            print(f"TLV {i}: Type 7 (Tracked Objects), Length = {tlv_length}")
            parse_tracked_objects(payload)
        else:
            print(f"TLV {i}: Type {tlv_type} (Unhandled), Length = {tlv_length}")

        offset += tlv_length

def read_packet_from_serial(ser):
    buffer = bytearray()
    while True:
        buffer += ser.read(2048)
        idx = find_magic_word(buffer)
        if idx == -1:
            continue

        if len(buffer) < idx + 40:
            continue

        header = parse_header(buffer[idx+8:idx+40])
        if not header:
            continue

        total_len = header['totalPacketLen']
        if len(buffer) < idx + total_len:
            continue  # wait for full packet

        print(f"\n--- Frame {header['frameNumber']} ---")
        parse_tlvs(buffer[idx+40:idx+total_len], header['numTLVs'])

        buffer = buffer[idx + total_len:]  # remove parsed data

def main():
    port = 'COM5'  # Adjust to your data UART port
    baud = 921600

    try:
        with serial.Serial(port, baudrate=baud, timeout=0.1) as ser:
            print(f"Listening on {port} at {baud} baud...")
            read_packet_from_serial(ser)
    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == '__main__':
    main()

