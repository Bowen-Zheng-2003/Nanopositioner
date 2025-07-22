import time
import serial
import ezdxf
from ezdxf.path import make_path

SERIAL_PORT = '/dev/tty.usbmodem1101'  # Change as needed for your system
BAUD_RATE = 115200
DXF_FILE = '/Users/bowenzheng/Downloads/Sketch2.dxf'

def extract_coordinates(doc, max_distance=1.0, scale=1.0):
    msp = doc.modelspace()
    coords = []
    for entity in msp:
        try:
            path = make_path(entity)
            for p in path.approximate(segments=20):
                coords.append((p.x * scale, p.y * scale))
        except (TypeError, AttributeError, ValueError, NotImplementedError):
            continue
    return coords

def send_coordinates_to_arduino(ser, coordinates):
    """Send coordinates to Arduino with proper protocol"""
    
    # Wait for Arduino to be ready
    print("Waiting for Arduino to be ready...")
    ready = False
    while not ready:
        if ser.in_waiting > 0:
            line = ser.readline().decode().strip()
            print("ARDUINO:", line)
            if "Ready for coordinates" in line or "Arduino ready" in line:
                ready = True
    
    # Start coordinate transmission
    print("Starting coordinate transmission...")
    ser.write(b"START_COORDS\n")
    time.sleep(0.1)  # Brief pause
    
    # Wait for ready confirmation
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode().strip()
            print("ARDUINO:", line)
            if "Ready to receive coordinates" in line:
                break
    
    # Send each coordinate
    for i, (x, y) in enumerate(coordinates):
        coord_str = f"{x},{y}\n"
        ser.write(coord_str.encode())
        print(f"Sent coordinate {i+1}/{len(coordinates)}: {x}, {y}")
        
        # Wait for acknowledgment
        ack_received = False
        timeout_count = 0
        while not ack_received and timeout_count < 50:  # 5 second timeout
            if ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                print(f"ARDUINO: {response}")
                if "ACK" in response:
                    ack_received = True
            else:
                time.sleep(0.1)
                timeout_count += 1
        
        if not ack_received:
            print(f"Warning: No acknowledgment received for coordinate {i+1}")
    
    # End coordinate transmission
    print("Ending coordinate transmission...")
    ser.write(b"END_COORDS\n")
    time.sleep(0.1)
    
    # Wait for final confirmation
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode().strip()
            print("ARDUINO:", line)
            if "coordinates total" in line:
                break

def main():
    try:
        # Read coordinates from DXF
        print(f"Reading DXF file: {DXF_FILE}")
        doc = ezdxf.readfile(DXF_FILE)
        xy_points = extract_coordinates(doc, scale=1000)  # Scale to microns

         # Add hardcoded first coordinate (origin or home position)
        HARDCODED_FIRST_COORD = (12500, 12500)  # Change these values as needed
        xy_points.insert(0, HARDCODED_FIRST_COORD)  # Insert at beginning
        
        print(f"Extracted {len(xy_points)} coordinates from DXF")
        
        if len(xy_points) == 0:
            print("No coordinates found in DXF file!")
            return
        
        # Show first few coordinates for verification
        print("First 5 coordinates:")
        for i, (x, y) in enumerate(xy_points[:5]):
            print(f"  {i+1}: ({x}, {y})")
        if len(xy_points) > 5:
            print(f"  ... and {len(xy_points) - 5} more")
        
        # Open serial connection to Arduino
        print(f"Connecting to Arduino on {SERIAL_PORT} at {BAUD_RATE} baud...")
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for Arduino to reset
        
        # Send coordinates
        send_coordinates_to_arduino(ser, xy_points)
        
        print("Coordinate transmission complete!")
        print("Arduino should now execute the movement sequence.")
        
        # Optionally, keep monitoring Arduino output
        monitor_arduino = input("Monitor Arduino output? (y/n): ").lower() == 'y'
        if monitor_arduino:
            print("Monitoring Arduino (Ctrl+C to stop):")
            try:
                while True:
                    if ser.in_waiting > 0:
                        line = ser.readline().decode().strip()
                        print("ARDUINO:", line)
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\nStopped monitoring.")
        
        ser.close()
        print("Connection closed.")
        
    except FileNotFoundError:
        print(f"Error: DXF file not found: {DXF_FILE}")
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}: {e}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
