#!/usr/bin/env python3
import os
import sys

# Import the necessary modules
import time
import numpy as np
import smbus2    
import argparse
from PIL import Image

def refresh_sensor(bus, device_address):
    if not set_test_mode(bus, device_address):
        print("Failed to set test mode. Aborting...")
        sys.exit(1)
    start_scan(bus, device_address)
    wait_scan(bus, device_address)
    print("Sensor refreshed")
    

def set_test_mode(bus, device_address):
    _test_mode = False
    DEVICE_MODE_REGISTER = 0x00  # Replace with the correct value for test mode
    TEST_MODE_VALUE = 0x04  << 4 # Replace with the correct value for test mode
    
    try:
        bus.write_byte_data(device_address, DEVICE_MODE_REGISTER, TEST_MODE_VALUE, force=True)
        _test_mode = True
        print("Test mode set")
        time.sleep(0.1)
        
    except IOError as e:
        print(f"Error writing to register 0x{DEVICE_MODE_REGISTER:02X}: {e}")
        
    return _test_mode
    

def set_working_mode(bus, device_address):
    DEVICE_MODE_REGISTER = 0x00  # Replace with the correct register address
    WORKING_MODE_VALUE = 0x00   # Replace with the correct value for test mode

    try:
        bus.write_byte_data(device_address, DEVICE_MODE_REGISTER, WORKING_MODE_VALUE, force=True)
        time.sleep(0.001)
    except IOError as e:
        print(f"Error writing to register 0x{DEVICE_MODE_REGISTER:02X}: {e}")
        
    while True:
        try:
            val = bus.read_byte_data(device_address, DEVICE_MODE_REGISTER)
            time.sleep(0.001)
            if val == 0x40:
                break
        except IOError as e:
            print(f"Error reading from register 0x{DEVICE_MODE_REGISTER:02X}: {e}")   
         
            
def start_scan(bus, device_address):
    try:
        bus.write_byte_data(device_address, 0x00, 0x02, force=True) #0xc0 used as address in github, 0x00 used in datasheet, although unsure if it would write?
        time.sleep(0.001)
    except IOError as e:
        print(f"I2C write failed: {e}")
        sys.exit(1)


def wait_scan(bus, device_address):
    while True:
        try:
            ret = bus.read_byte_data(device_address, 0x00)
            time.sleep(0.001)
            if ret == 0x00 or ret == 0x40: # The scan register value during a scan is 0x02, when it's done it returns 0x00
                return
            else:
                print(f"Got 0x{ret:02X} waiting...")
                time.sleep(0.003)
        except IOError as e:
            print(f"I2C read failed at wait_scan: {e}")

    

# Read the weight byte from the specified register address
def dump_weight_data(bus, address, register):
    try:
        weight = bus.read_byte_data(address, register)
        return weight
            
    except IOError as e:
        print(f"An I/O error occurred while reading register {register}: {e.strerror}.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}. Continuing...")
    return None


# Read a block of data from the specified register address range using an amount of registers
def read_raw_data(start_address,num_registers,i2c_address, bus):
    time.sleep(0.5)
    return bus.read_i2c_block_data(i2c_address, start_address, num_registers)
    
    
# Read a block of data from the specified register address
def dump_raw_data(mode, filename, data):
    print(f"Dumping {mode} data into {filename}...") 
    with open(filename, 'wb') as f:  # Use 'wb' for writing in binary mode
        f.write(data)  # Assuming 'data' is a bytes object


def check_and_repair_first_byte(bytes):
    if bytes[0] >= 0x10:
        # Invalid X data - zeroing high nibble and running anyway...
        bytes[0] = bytes[0] & 0x0F  # Zero the high nibble
        # Repaired first byte: bytes[0]
    return bytes


def read_cac_registers(bus, device_address):
    row_data = []
    col_data = []
    row_cac_registers = (0x78, 0x9F) #ROW0_CAC -> ROW39_CAC 
    col_cac_registers = (0xA0, 0xBE) #COL0_CAC -> COL29_CAC
    ranges = [row_cac_registers, col_cac_registers]
    for i, (start_register, end_register) in enumerate(ranges):
            for register in range(start_register, end_register + 1):
                try:
                    value = bus.read_byte_data(device_address, register)
                    time.sleep(0.1)
                    if i == 0:
                        row_data.append(f'{hex(register)}:{hex(value)}')
                    else:
                        col_data.append(f'{hex(register)}:{hex(value)}')
                except IOError as e:
                    print(f"Error reading register 0x{register:02X}: {e}. Retrying...")
                    time.sleep(0.1)
    return row_data, col_data

def read_data(bus, device_address):
    print("Reading registers...")
    rows = 22
    cols = 12
    datarows = [[0 for _ in range(cols)] for _ in range(rows)]
    retval = []


    for r in range(rows):
        try:
            bus.write_byte_data(device_address, 0x01, r) # Set the row to read starting with 0 to 22
            time.sleep(0.001)
        except IOError as e:
            print(f"Error writing to register 0x01: {e}. Retrying...")

        values = None
        try:
            values = bus.read_i2c_block_data(device_address, 0x12, 2*cols)
            time.sleep(0.001)
        except IOError as e:
            print(f"Error reading register 0x10: {e}. Retrying...")
        
        if values is not None:
            for c in range(cols):
                try: 
                    val = values[2*c] << 8 | (values[2*c+1])
                    datarows[r][c] = val
                    retval.append(val)
                except Exception as e:
                    print(f"Error reading register 0x{10 + 2*c:02X}: {e}. Retrying... Infinite Loop?")
    
    return datarows, retval

def read_image(bus, device_address):
    rows = 22
    cols = 12
    
    datarows, retval = read_data(bus, device_address)
    
    img_data = np.array(datarows).astype(np.uint8)
    img = Image.fromarray(img_data)
    img_flipped = img.transpose(Image.TRANSPOSE)
    
    signal_min = np.min(img_data)
    signal_max = np.max(img_data)
    
    img_array = np.array(img_flipped)
    img_scaled = ((img_array - signal_min) / (signal_max - signal_min)) * 255.0
    
    img_scaled = Image.fromarray(img_scaled.astype(np.uint8))
    img_scaled.save('output.bmp')


def main():
    
    # The '1' argument refers to the I2C bus number; your device might be on bus 0 or another bus.
    bus = smbus2.SMBus(1)
    # Specify the I2C address of the touchscreen controller
    # This is usually provided in the device's datasheet.
    TOUCHSCREEN_I2C_ADDRESS = 0x38  # This is the address for the FT56X4 chip on the Raspberry Pi Display v1.1 (2015)
    START_ADDRESS = 0x10
    END_ADDRESS = 0x4B
    REGISTER_LENGTH = END_ADDRESS - START_ADDRESS + 1
    # Specify the register addresses that you want to read from
    # You would replace these with the actual register addresses from your device's datasheet.  

    # Read registers from 0x00 up to 0x3E   
    weight_registers = [0x07, 0x0D, 0x13, 0x19, 0x1F, 0x25, 0x2B, 0x31, 0x37, 0x3D]
    weights = {}
    
    
    parser = argparse.ArgumentParser(description='Dump raw data from the touchscreen.')
    parser.add_argument('mode', choices=['background', 'frames','scan'], help='The mode to dump data: "background" or "frames" or "scan"')
    parser.add_argument('count', type=int, nargs='?',help='Number of times to dump data')

    # Parse arguments
    args = parser.parse_args()
    
    #set_working_mode(bus, TOUCHSCREEN_I2C_ADDRESS)
    # Check the mode and call the function
    if args.mode == 'background' or args.mode == 'frames':
        for i in range(1, args.count + 1):
            filename = f"{args.mode}_{i}.raw"
            
            #Delete the files if it already exists
            if os.path.exists(filename):
                os.remove(filename)
                
            # Reading the available registers
            datarows, retval = read_data(bus, TOUCHSCREEN_I2C_ADDRESS)
            print(retval)
            dump_raw_data(args.mode, filename, bytes(retval))
            
    if args.mode == 'scan':
        refresh_sensor(bus, TOUCHSCREEN_I2C_ADDRESS)
        while True:
            datarows, retval = read_data(bus, TOUCHSCREEN_I2C_ADDRESS)
            print(f"{datarows}")
        
    else:
        print(f"Unknown mode: {args.mode}")
        sys.exit(1)

    
if __name__ == "__main__":
    main()

