import serial
import numpy as np
import csv
import time
import sys
from collections import deque

def readLidarData(port, numPackets=38*3):
    '''
    Function that Obtains a single reading from the LD19 Lidar Sensor
    Inputs:
    port - Serial port to read data from
    numPackets - Number of data packets to be returned
    Outputs:
    scan - Dictionary containing all the scan information
    '''
    header = b'\x54'
    baudrate=230400
    ser = None

    try:
        # Open Serial Communication with Given Port and Baudrate
        ser = serial.Serial(port, baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1)

        # Read one byte at a time until the header byte is found
        while True:
            if ser.in_waiting > 0:
                byte = ser.read(1)
                if byte == header:
                    break
        
        # Read rest of the data packet from the lidar sensor and store in an np array
        data = np.frombuffer(header + ser.read(46) + ser.read(47 * (numPackets - 1)), dtype=np.uint8).reshape((-1,47))
        
        # Filter data by checking checksums for invalid data
        data = np.where(lambda p: check_packet(p[::-2]) == p[-1], data)

        # Create a dictionary to store the scan data
        scan = {}
        
        # Store speed of lidar motor in deg/s
        scan['speed'] = data[:,3].astype(np.uint16) << 8 | data[:,2]
        
        # Calculate and store start and end angle 
        scan['start_angle'] = (data[:,5].astype(np.uint16) << 8 | data[:,4]) * 0.01
        scan['end_angle'] = (data[:,43].astype(np.uint16) << 8 | data[:,42]) * 0.01

        # Calculate and store all ranges in an np array
        scan['ranges'] = np.array([data[:,7].astype(np.uint16) << 8 | data[:,6],
                                   data[:,10].astype(np.uint16) << 8 | data[:,9],
                                   data[:,13].astype(np.uint16) << 8 | data[:,12],
                                   data[:,16].astype(np.uint16) << 8 | data[:,15],
                                   data[:,19].astype(np.uint16) << 8 | data[:,18],
                                   data[:,22].astype(np.uint16) << 8 | data[:,21],
                                   data[:,25].astype(np.uint16) << 8 | data[:,24],
                                   data[:,28].astype(np.uint16) << 8 | data[:,27],
                                   data[:,31].astype(np.uint16) << 8 | data[:,30],
                                   data[:,34].astype(np.uint16) << 8 | data[:,33],
                                   data[:,37].astype(np.uint16) << 8 | data[:,36],
                                   data[:,40].astype(np.uint16) << 8 | data[:,39]])

        # Extract and store intensity values from lidar scan
        scan['intensities'] = data[:,8:42:3].T
        
    # Exception Statements for connection failures 
    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Exiting...")
    # Close Serial Port once function terminates
    finally:
        if ser and ser.is_open:
            ser.close()

    return scan


CrcTable = [
0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8 ]

def check_packet(packet):
    crc = 0
    for b in packet:
        crc = CrcTable[(crc ^ b) & 0xff]
    return crc

def save_lidar_data_to_csv(filename):
    scan = readLidarData('/dev/ttyUSB0')
    
    if scan:
        angles = np.linspace(scan['start_angle'], scan['end_angle'], 12)
        x_coords = (scan['ranges'] * np.cos(np.deg2rad(angles))).reshape(-1, order='F') / 1000
        y_coords = (scan['ranges'] * np.sin(np.deg2rad(angles))).reshape(-1, order='F') / 1000
        print(np.vstack((x_coords, y_coords).T))
        # Write data to CSV file
        with open(filename, 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['x', 'y'])
            for x, y in zip(x_coords, y_coords):
                csvwriter.writerow([float(x), float(y)])  # Ensure data is saved as plain floats
        print(f"Data saved to {filename}")
    else:
        print("No data to save.")

# Save data to CSV
save_lidar_data_to_csv('lidar_data.csv')
