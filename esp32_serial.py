import serial
import time

# Configuration parameters
port = '/dev/ttyACM0'#'/dev/ttyUSB0'  # Replace 'COMx' with the correct port number
baud_rate = 115200
timeout = 2  # Timeout for serial communication

# Initialize serial connection
try:
    ser = serial.Serial(port, baud_rate, timeout=timeout)
    print("Connected to ESP32 on port " + port)
except Exception as e:
    print("Failed to connect on port: " + str(port))
    print(str(e))
    exit()

def send_command(command):
    """Send a command to the ESP32 via serial."""
    ser.write((command + '\n').encode())  # Command must end with a newline character
    time.sleep(0.1)  # Give some time for the ESP32 to respond
    while ser.in_waiting:
        print(ser.readline().decode().strip())  # Print the ESP32's response

# Example usage
try:
    while True:
        cmd = input("Enter command (OPEN (v), CLOSE (c), MAG_ON (m), MAG_OFF (n), or q to quit): ")
        if cmd.lower() == 'q':
            break
        elif cmd in ['v', 'c', 'm', 'n']:
            send_command(cmd)
        else:
            print("Invalid command. Please try again.")
finally:
    ser.close()
    print("Serial connection closed.")