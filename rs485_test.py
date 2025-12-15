import serial
from struct import pack, unpack
import time
import sys

# --- 1. Protocol Definitions ---
CMD_HEADER                     = 0x3E
CMD_ASK_ERROR_STATUS           = 0x9A        # New command: Read motor status 1 and error flag command

# --- 2. Checksum Function ---

def calculate_checksum(data: bytes) -> int:
    """Calculates the 8-bit sum checksum (mod 256) for a given byte array."""
    checksum = sum(data) % 256
    return checksum

# --- 3. Communication Function ---

def get_motor_error_status(serial_port: serial.Serial, motor_id: int = 0x00) -> bytes:
    """
    Sends the request to read the motor status and error flag (0x9A).
    Returns the raw response bytes for analysis.
    """
    data_length = 0x00  # No data block for a simple read command
    
    # 1. Build the 4-byte header for CRC calculation
    header_data = bytes([CMD_HEADER, CMD_ASK_ERROR_STATUS, motor_id, data_length])
    
    # 2. Calculate the Header CRC
    header_crc = calculate_checksum(header_data)
    
    # 3. Construct the full 5-byte request packet
    request_packet = pack('<BBBBB', 
                          CMD_HEADER, 
                          CMD_ASK_ERROR_STATUS, 
                          motor_id, 
                          data_length, 
                          header_crc)

    print(f"-> Sending Status Request (Hex): {' '.join(f'{b:02X}' for b in request_packet)}")

    # 4. Send the request
    serial_port.write(request_packet)

    # 5. Wait for the response. We will try to read 8 bytes, which is the 
    # expected length for simple data + checksum response.
    serial_port.timeout = 0.5 
    response = serial_port.read(8)

    if len(response) == 0:
        raise TimeoutError(f"Motor did not respond (expected 8 bytes, got {len(response)}).")

    # If we get *some* data but not 8 bytes, we can still analyze it.
    if len(response) < 8:
        print(f"WARNING: Received {len(response)} bytes, expected 8.")

    return response

# --- 4. Main Execution Block ---

# Configuration
SERIAL_PORT_NAME = "/dev/ttyUSB0"
BAUDRATE = 115200
MOTOR_ID = 0x01 
# The Header CRC for 0x9A command is: (0x3E + 0x9A + 0x00 + 0x00) % 256 = 0xD8.
# (Hex calculation: 3E + 9A = D8)

try:
    # 1. Initialize the serial port
    ser = serial.Serial(
        port        =   SERIAL_PORT_NAME,
        baudrate    =   BAUDRATE,
        parity      =   serial.PARITY_NONE,
        stopbits    =   serial.STOPBITS_ONE,
        bytesize    =   serial.EIGHTBITS,
        timeout     =   0.5 
    )
    
    ser.flushInput()
    ser.flushOutput()
    
    # 2. Get the status
    print(f"Attempting connection on {SERIAL_PORT_NAME} at {BAUDRATE} baud...")
    raw_response = get_motor_error_status(ser, motor_id=MOTOR_ID)

    print("\n--- ✅ RESPONSE RECEIVED ---")
    print(f"<- Received Raw Response (Hex): {' '.join(f'{b:02X}' for b in raw_response)}")
    
    if len(raw_response) == 8:
        # Assuming the error/status data is in bytes 6 and 7 (index 5 and 6)
        status_data = raw_response[5:7]
        status_value = unpack("<H", status_data)[0]
        
        print(f"\nExtracted Status/Error Code (Decimal): {status_value}")
        print("Note: You must consult the motor manual to interpret this code.")
    else:
        print("\nCould not extract status code due to incomplete response length.")

    print("\nCommunication established (at least one way).")
    print("Wiring and serial settings are likely correct if the response is non-zero length.")

except serial.SerialException as e:
    print("\n--- ❌ SERIAL PORT ERROR ---")
    print(f"Could not open serial port {SERIAL_PORT_NAME}. Error: {e}")
    sys.exit(1)
    
except TimeoutError as e:
    print("\n--- ❌ TIMEOUT ERROR ---")
    print("Connection established, but the motor did not reply at all (0 bytes).")
    print("Please re-check your termination resistor setting on BOTH ends!")
    print(f"Error details: {e}")
    sys.exit(1)

except Exception as e:
    print(f"\n--- ❌ GENERIC ERROR ---")
    print(f"An unexpected error occurred: {e}")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")