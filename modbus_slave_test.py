import serial
import random
import os

def parse_modbus_frame(data):
    """Parse a Modbus RTU frame."""
    if len(data) < 8:  # Minimum Modbus RTU frame length
        print("Invalid Modbus frame: Too short")
        return

    # Extract Modbus frame components
    slave_id = data[0]
    function_code = data[1]
    address = (data[2] << 8) | data[3]  # Combine high and low bytes for the address
    value = (data[4] << 8) | data[5]  # Combine high and low bytes for the value
    crc_received = (data[-2] | (data[-1] << 8))  # Combine low and high bytes for CRC

    # Print parsed data
    print("Parsed Modbus Frame:")
    print(f"  Slave ID: {slave_id}")
    print(f"  Function Code: {function_code}")
    print(f"  Address: {address:#06x}")
    print(f"  Value: {value:#06x}")
    print(f"  CRC Received: {crc_received:#06x}")

    # Validate CRC (optional, requires a CRC calculation function)
    crc_calculated = calculate_crc(data[:-2])  # Exclude the last two CRC bytes
    if crc_calculated == crc_received:
        print("CRC validation passed.")
    else:
        print(f"CRC validation failed. Calculated: {crc_calculated:#06x}")

def calculate_crc(data):
    """Calculate the Modbus CRC16 checksum."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def generate_modbus_response(data):
    """Generate a Modbus RTU response based on the received request."""
    if len(data) < 8:  # Minimum Modbus RTU frame length
        print("Invalid Modbus frame: Too short")
        return None

    # Extract Modbus frame components
    slave_id = data[0]
    function_code = data[1]
    address = (data[2] << 8) | data[3]

    # Generate a response based on the function code
    if function_code == 0x06:  # Write Single Register
        value = (data[4] << 8) | data[5]
        # Echo back the same request as the response
        response = data[:6]  # Slave ID, Function Code, Address, and Value
    elif function_code == 0x03:  # Read Holding Registers
        num_registers = (data[4] << 8) | data[5]  # Number of registers to read

        # Simulated data store for holding registers
        holding_registers = [random.randint(0x0000, 0x000A) for _ in range(10)]  # Populate with random values

        if address + num_registers > address + len(holding_registers):
        # if address + num_registers > len(holding_registers):
            print("Invalid address or number of registers")
            return None
        
        # Retrieve the requested registers
        # register_values = holding_registers[address:address + num_registers]
        register_values = holding_registers[0:num_registers]

        # Construct the response
        byte_count = len(register_values) * 2  # Each register is 2 bytes
        response = bytes([slave_id, function_code, byte_count])
        for value in register_values:
            response += value.to_bytes(2, byteorder="big")  # Convert each register to 2 bytes
    else:
        print(f"Unsupported function code: {function_code}")
        return None

    # Calculate CRC for the response
    crc = calculate_crc(response)
    crc_lsb = crc & 0xFF
    crc_msb = (crc >> 8) & 0xFF

    # Append CRC to the response
    response += bytes([crc_lsb, crc_msb])
    return response

def modbus_main_handler(ser):
    print("Listening for data...")
    while True:
        # Read data from the serial port
        data = ser.read(1024)  # Read up to 100 bytes or until timeout
        if data:
            print(f"Raw Data Received: {data.hex()}")  # Print raw data in hexadecimal format

            # Parse the Modbus frame
            parse_modbus_frame(data)

            # Generate Modbus response
            response = generate_modbus_response(data)
            if response:
                print(f"Response sent: {response.hex()}")
                # Send the response back to the sender
                ser.write(response)

def main():
    # Change the current working directory to so that opening relative paths/files will work
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)

    try:
        # Open the serial port
        ser = serial.Serial(port="COM9", baudrate=9600, parity="N", stopbits=1, bytesize=8, timeout=1)
        print(f"Serial port {ser.port} opened successfully.")
        modbus_main_handler(ser)
    except Exception as e:
        print(f"Error opening or reading from serial port: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == '__main__':
    main()