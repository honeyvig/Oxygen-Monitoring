# Oxygen-Monitoring
We have an oxygen sensor need to read it's data over UART SERIAL to pi and send data to plc via rs485
--
To accomplish the task of reading data from an oxygen sensor via UART (Serial) on a Raspberry Pi and sending the data to a PLC via RS485, we can break the solution into two main parts:

    Read data from the oxygen sensor via UART on Raspberry Pi.
    Send the sensor data to the PLC via RS485 communication.

Prerequisites:

    Oxygen Sensor: It should have a UART interface to send data.
    Raspberry Pi: A device with UART (Serial) capabilities to communicate with the oxygen sensor.
    RS485 to USB adapter: This will allow the Raspberry Pi to communicate via RS485 to the PLC.
    PLC: Your programmable logic controller that is capable of reading data via RS485.

You’ll need:

    Python installed on your Raspberry Pi.
    A serial library (pyserial) to handle UART communication.
    A library or tool to manage RS485 communication (you can use pyModbus or direct serial communication).
    minicom or screen for checking the sensor output via UART.

Steps:
1. Install Required Libraries

Install pyserial for UART communication with the sensor and pyModbus (or minimalmodbus if you prefer) for communicating with the PLC via RS485.

sudo apt-get update
sudo apt-get install python3-serial
sudo pip3 install pyserial pyModbus

2. Read Data from the Oxygen Sensor over UART

Assume the oxygen sensor is connected to the Raspberry Pi via a UART (Serial) connection on a specific port (e.g., /dev/ttyAMA0 or /dev/ttyS0).

Here’s the Python code to read data from the oxygen sensor over UART:

import serial
import time

# Set up the serial connection to the oxygen sensor (e.g., on /dev/ttyS0)
# Make sure the port and baud rate match your sensor specifications
oxygen_sensor_serial = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=1)

def read_oxygen_data():
    """Reads data from the oxygen sensor."""
    try:
        # Send command to oxygen sensor if required (e.g., requesting data)
        # oxygen_sensor_serial.write(b'GET DATA COMMAND\n')

        # Read incoming data (assuming it's a simple number or string)
        data = oxygen_sensor_serial.readline().decode('utf-8').strip()
        print("Oxygen Sensor Data: ", data)
        return data
    except Exception as e:
        print(f"Error reading from oxygen sensor: {e}")
        return None

# Continuously read data every 1 second
while True:
    oxygen_data = read_oxygen_data()
    if oxygen_data:
        # Process or send the data further (send it to PLC)
        pass
    time.sleep(1)

    This code reads data from the oxygen sensor and prints it to the console.
    Adjust the baud rate (9600 in the example) and the port (/dev/ttyS0) according to your specific oxygen sensor.

3. Send Data to the PLC via RS485

Now that we have the data from the oxygen sensor, we can send it to the PLC over RS485. You’ll need to use an RS485 USB-to-Serial converter connected to the Raspberry Pi.

Here’s an example of how to send data using the Modbus RTU protocol via RS485:

from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import time

# Set up the Modbus RS485 client
client = ModbusClient(method='rtu', port='/dev/ttyUSB0', baudrate=9600, timeout=1)

# Connect to the PLC
if not client.connect():
    print("Unable to connect to the PLC via RS485.")
else:
    print("Connected to the PLC.")

def send_data_to_plc(data):
    """Send data to the PLC using Modbus RTU over RS485."""
    try:
        # Assume we are writing data to a holding register (e.g., address 1)
        # The data from the sensor should be converted into an integer or appropriate type
        # Example: sending the oxygen value as a 16-bit register value
        oxygen_value = int(float(data))  # Convert to an integer (you may need to scale or process it)
        
        # Send the data to the PLC - address 1 (holding register 1)
        # This writes the oxygen value to a specific register
        response = client.write_register(1, oxygen_value, unit=1)
        
        if response.isError():
            print(f"Error sending data to PLC: {response}")
        else:
            print(f"Successfully sent data to PLC: {data}")
    except Exception as e:
        print(f"Error sending data to PLC: {e}")

# Continuously read oxygen data and send it to PLC
while True:
    oxygen_data = read_oxygen_data()
    if oxygen_data:
        send_data_to_plc(oxygen_data)
    time.sleep(1)

Explanation of Code:

    Modbus RTU over RS485:
        We use the pymodbus library to communicate with the PLC over RS485 using Modbus RTU.
        The ModbusClient connects to the PLC via the /dev/ttyUSB0 port (the port where your RS485-to-USB converter is connected).
        The function send_data_to_plc sends the oxygen sensor data to a Modbus holding register on the PLC.
    Oxygen Data:
        We assume that the oxygen sensor returns a numeric value, which we convert to an integer (int(float(data))) and send to the PLC. You may need to adjust how the data is formatted based on your sensor's output.

4. Test and Run the Code

Once everything is set up:

    Make sure your oxygen sensor is correctly connected to the Raspberry Pi’s UART pins (TX, RX, and GND).
    Ensure your RS485-to-USB adapter is connected to the Raspberry Pi and the PLC.
    Run the Python script to start reading data from the sensor and send it to the PLC.

python3 oxygen_sensor_to_plc.py

The script will continuously read data from the oxygen sensor, print the data to the console, and send it to the PLC over RS485 using the Modbus RTU protocol.
Conclusion:

    Raspberry Pi acts as the interface to read data from the oxygen sensor using UART and sends the data to the PLC via RS485.
    The example uses the Modbus RTU protocol to communicate with the PLC. You can modify it according to your specific PLC communication requirements.
    Adjust sensor reading, error handling, and the format of the data sent to the PLC as necessary.

This should give you a complete starting point to interface your oxygen sensor with a Raspberry Pi and send the data to a PLC over RS485.
