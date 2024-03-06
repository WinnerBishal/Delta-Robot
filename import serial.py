import serial
import time

# Initialize serial connection (COM port and baud rate must match Arduino's)
ser = serial.Serial('COM7', 9600)
time.sleep(2) # Wait for the connection to establish

def send_command(command):
    print(f"Sending: {command}")
    ser.write((command + '\n').encode())
    while True:
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            print(f"Arduino says: {response}")
            break

# Example command sequence
for position in range(0, 10000, 1000):
    send_command(str(position))
    time.sleep(1) # Adjust based on your motor's speed and required delay

ser.close() # Close the serial connection
