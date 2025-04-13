#!/usr/bin/python3

import os
import time
import threading
import queue
import customtkinter as ctk
import serial.tools.list_ports  # Import to list available serial ports
import configparser  # Import configparser to handle config.ini
from pymodbus.server import StartSerialServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSparseDataBlock, ModbusSlaveContext, ModbusServerContext  # Add this import
import serial  # Import pyserial for sending Modbus data
import traceback  # Import traceback for detailed error logging
from pymodbus.framer.rtu import FramerRTU  # Import the RTU framer
import json  # Import json for saving relay states


# Configuration for Modbus
class RelayDevice:
    def __init__(self, app, baudrate=9600, parity='N', stopbits=1, bytesize=8):
        self.slave_id = 1
        self.start_address = 0x0032  # Starting address in hexadecimal
        self.store = CallbackDataBlock(
            {self.start_address + i: 0 for i in range(16)},  # Initialize 16 relays
            lambda address, value: relay_callback(address, value, app)  # Pass the app instance
        )

        self.baudrate = baudrate
        self.parity = parity
        self.stopbits = stopbits
        self.bytesize = bytesize
        self.serial_port = "COM9"  # Default serial port

        # Use the serial queue from the app
        self.serial_queue = app.serial_queue

        self.start_rtu_server()

    def start_rtu_server(self):
        # Use ModbusSlaveContext instead of ModbusContext
        context = ModbusSlaveContext(
            di=None,
            co=None,
            hr=self.store,  # Use holding registers for relay control
            ir=None,
        )
        # Wrap the context in a ModbusServerContext
        server_context = ModbusServerContext(slaves=context, single=True)
        threading.Thread(target=self.run_rtu_server, args=(server_context,), daemon=True).start()

    def run_rtu_server(self, context):
        print(f"Starting Modbus RTU server on {self.serial_port}...")
        try:
            StartSerialServer(
                context,
                port=self.serial_port,
                baudrate=self.baudrate,
                parity=self.parity,
                stopbits=self.stopbits,
                bytesize=self.bytesize,
                framer="rtu",  # Use the correct string key for RTU framer
                handle_local_echo=False,  # Disable local echo
                custom_functions=None,  # Default Modbus functions
                on_data_received=self.log_serial_data  # Add a callback for logging raw data
            )
        except Exception as e:
            print(f"Error starting Modbus RTU server: {e}")
            traceback.print_exc()  # Print the full exception traceback

    def log_serial_data(self, data):
        """Log and parse raw data received from the serial port."""
        print(f"Raw data received from serial port: {data.hex()}")

        # Parse the Modbus frame
        if len(data) >= 8:  # Minimum Modbus RTU frame length
            slave_id = data[0]
            function_code = data[1]
            address = (data[2] << 8) | data[3]
            value = (data[4] << 8) | data[5]
            crc_received = (data[-2] | (data[-1] << 8))

            # Log parsed data
            print(f"Parsed Modbus Frame:")
            print(f"  Slave ID: {slave_id}")
            print(f"  Function Code: {function_code}")
            print(f"  Address: {address:#06x}")
            print(f"  Value: {value:#06x}")
            print(f"  CRC Received: {crc_received:#06x}")
        else:
            print("Invalid Modbus frame received.")

    def configure_serial(self, baudrate, parity, stopbits, bytesize):
        self.baudrate = baudrate
        self.parity = parity
        self.stopbits = stopbits
        self.bytesize = bytesize
        self.restart_rtu_server()

    def restart_rtu_server(self):
        print("Restarting RTU server with new parameters")
        self.start_rtu_server()

    def send_modbus_data(self, address, value):
        """Send Modbus data to control relays or communicate states."""
        try:
            # Construct the Modbus frame
            slave_id = self.slave_id
            function_code = 0x06  # Function code for writing a single register
            address_high = (address >> 8) & 0xFF
            address_low = address & 0xFF
            value_high = (value >> 8) & 0xFF
            value_low = value & 0xFF

            # Create the Modbus frame
            modbus_frame = bytes([slave_id, function_code, address_high, address_low, value_high, value_low])

            # Calculate CRC
            crc = calculate_crc(modbus_frame)
            crc_lsb = crc & 0xFF  # Least significant byte
            crc_msb = (crc >> 8) & 0xFF  # Most significant byte

            # Append CRC to the frame
            modbus_frame += bytes([crc_lsb, crc_msb])

            # Put the Modbus frame into the serial queue
            self.serial_queue.put(modbus_frame)
            print(f"Enqueued Modbus frame: {modbus_frame.hex()}")

        except Exception as e:
            print(f"Error constructing Modbus data: {e}")

# GUI Application
class RelayApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Virtual Modbus Slave")
        self.geometry("860x680")

        # Initialize relay states
        self.relay_states = [False] * 16  # Default to all relays OFF
        self.relay_buttons = []

        # Load configuration from config.ini
        self.config = configparser.ConfigParser()
        self.config.read("config.ini")
        self.serial_config = self.config["SerialConfig"]

        # Load Switching Actuator Data
        self.load_switching_actuator_data()

        # Create main frames for layout
        self.config_frame = ctk.CTkFrame(self, width=300)
        self.config_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        self.relay_frame = ctk.CTkFrame(self, width=500)
        self.relay_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        # Configure grid weights
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # Configuration Section (Left)
        ctk.CTkLabel(self.config_frame, text="Serial Configuration:").pack(pady=10)

        ctk.CTkLabel(self.config_frame, text="Slave ID:").pack(pady=2)
        self.slave_id_entry = ctk.CTkEntry(
            self.config_frame, placeholder_text="Enter Slave ID (e.g., 1)"
        )
        self.slave_id_entry.insert(0, self.serial_config.get("slave_id", "1"))
        self.slave_id_entry.pack(pady=5)

        ctk.CTkLabel(self.config_frame, text="Serial Port:").pack(pady=2)
        self.serial_ports = self.get_serial_ports()
        self.serial_port_menu = ctk.CTkOptionMenu(
            self.config_frame, values=self.serial_ports
        )
        self.serial_port_menu.set(self.serial_config.get("serial_port", "COM9"))
        self.serial_port_menu.pack(pady=5)

        ctk.CTkLabel(self.config_frame, text="Baud Rate:").pack(pady=2)
        self.baudrate_entry = ctk.CTkEntry(
            self.config_frame, placeholder_text="Enter Baud Rate (e.g., 9600)"
        )
        self.baudrate_entry.insert(0, self.serial_config.get("baudrate", "9600"))
        self.baudrate_entry.pack(pady=5)

        ctk.CTkLabel(self.config_frame, text="Parity:").pack(pady=2)
        self.parity_entry = ctk.CTkEntry(
            self.config_frame, placeholder_text="Enter Parity (e.g., N)"
        )
        self.parity_entry.insert(0, self.serial_config.get("parity", "N"))
        self.parity_entry.pack(pady=5)

        ctk.CTkLabel(self.config_frame, text="Stop Bits:").pack(pady=2)
        self.stopbits_entry = ctk.CTkEntry(
            self.config_frame, placeholder_text="Enter Stop Bits (e.g., 1)"
        )
        self.stopbits_entry.insert(0, self.serial_config.get("stopbits", "1"))
        self.stopbits_entry.pack(pady=5)

        ctk.CTkLabel(self.config_frame, text="Byte Size:").pack(pady=2)
        self.bytesize_entry = ctk.CTkEntry(
            self.config_frame, placeholder_text="Enter Byte Size (e.g., 8)"
        )
        self.bytesize_entry.insert(0, self.serial_config.get("bytesize", "8"))
        self.bytesize_entry.pack(pady=5)

        self.start_button = ctk.CTkButton(
            self.config_frame, text="Start Server", command=self.start_server
        )
        self.start_button.pack(pady=10)

        self.update_serial_button = ctk.CTkButton(
            self.config_frame, text="Update Serial Config", command=self.update_serial_config
        )
        self.update_serial_button.pack(pady=10)

        self.feedback_label = ctk.CTkLabel(self.config_frame, text="", text_color="green")
        self.feedback_label.pack(pady=5)

        # Relay Buttons Section (Right)
        ctk.CTkLabel(self.relay_frame, text="Relay Controls:").grid(row=0, column=0, columnspan=2, pady=10)

        self.relay_states = [False] * 16
        self.relay_buttons = []

        # Create 16 relay buttons in an 8x2 grid
        for i in range(self.relay_quantity):
            row = i // 2
            col = i % 2
            btn = ctk.CTkButton(
                self.relay_frame,
                text=f"Relay {i+1}",
                command=lambda i=i: self.toggle_relay(i),
                border_width=2  # Set the border width to make it more bold
            )
            # Set padding for the button
            btn.grid(row=row + 1, column=col, padx=10, pady=10, sticky="nsew")  # Adjust padx and pady as needed
            self.relay_buttons.append(btn)

        # Configure grid weights for relay buttons
        for r in range(8):  # 8 rows
            self.relay_frame.grid_rowconfigure(r + 1, weight=1)
        for c in range(2):  # 2 columns
            self.relay_frame.grid_columnconfigure(c, weight=1)

        self.switch_button = ctk.CTkButton(
            self.relay_frame,
            text="Manual Switch",
            command=self.manual_switch_press,
            border_width=2  # Set the border width to make it more bold
        )
        self.switch_button.grid(row=9, column=0, columnspan=2, pady=10)

        self.led_indicator = ctk.CTkLabel(self.relay_frame, text="LED OFF", text_color="red")
        self.led_indicator.grid(row=10, column=0, columnspan=2, pady=10)

        # Long press variables
        self.press_start_time = None

        # Create a queue for communication between threads
        self.serial_queue = queue.Queue()

        # Start the serial communication thread
        self.serial_thread = threading.Thread(target=self.handle_serial_communication, daemon=True)
        self.serial_thread.start()

        # Schedule serial data processing
        self.after(100, self.process_serial_data)

    def get_serial_ports(self):
        """Retrieve a list of available serial ports."""
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports] or ["COM9"]  # Default to COM9 if no ports found
        print(f"Available ports: {', '.join(port_list)}")  # Print ports in one line
        
        return [port.device for port in ports] or ["COM9"]  # Default to COM9 if no ports found

    def start_server(self):
        serial_port = self.serial_port_menu.get()
        slave_id = self.slave_id_entry.get()
        slave_id = int(slave_id) if slave_id.isdigit() else 1  # Default to 1 if invalid
        baudrate = self.baudrate_entry.get()
        baudrate = int(baudrate) if baudrate.isdigit() else 9600  # Use default 9600 if invalid
        parity = self.parity_entry.get()
        stopbits = self.stopbits_entry.get()
        stopbits = int(stopbits) if stopbits.isdigit() else 1  # Default to 1 if invalid
        bytesize = self.bytesize_entry.get()
        bytesize = int(bytesize) if bytesize.isdigit() else 8  # Default to 8 if invalid

        # Pass self (app instance) to RelayDevice
        self.relay_device = RelayDevice(self, baudrate, parity, stopbits, bytesize)
        self.relay_device.slave_id = slave_id  # Set the selected slave ID
        self.relay_device.serial_port = serial_port  # Set the selected serial port
        self.feedback_label.configure(text="Server started with Slave ID: " + str(slave_id))

    def update_serial_config(self):
        """Update the serial configuration and save to config.ini."""
        if self.relay_device:
            try:
                serial_port = self.serial_port_menu.get()
                slave_id = self.slave_id_entry.get()
                slave_id = int(slave_id) if slave_id.isdigit() else 1  # Default to 1 if invalid
                baudrate = self.baudrate_entry.get()
                baudrate = int(baudrate) if baudrate.isdigit() else 9600  # Use default 9600 if invalid
                parity = self.parity_entry.get()
                stopbits = self.stopbits_entry.get()
                stopbits = int(stopbits) if stopbits.isdigit() else 1  # Default to 1 if invalid
                bytesize = self.bytesize_entry.get()
                bytesize = int(bytesize) if bytesize.isdigit() else 8  # Default to 8 if invalid

                # Update the relay device configuration
                self.relay_device.slave_id = slave_id
                self.relay_device.configure_serial(baudrate, parity, stopbits, bytesize)
                self.relay_device.serial_port = serial_port

                # Save the updated configuration to config.ini
                self.config["SerialConfig"] = {
                    "slave_id": str(slave_id),
                    "baudrate": str(baudrate),
                    "parity": parity,
                    "stopbits": str(stopbits),
                    "bytesize": str(bytesize),
                    "serial_port": serial_port,
                }
                with open("config.ini", "w") as configfile:
                    self.config.write(configfile)

                self.feedback_label.configure(
                    text="Serial configuration updated successfully with Slave ID: " + str(slave_id)
                )
            except ValueError:
                self.feedback_label.configure(
                    text="Invalid values. Please check your input.", text_color="red"
                )

    def toggle_relay(self, index):
        # Check if the relay device is initialized
        if not hasattr(self, "relay_device") or self.relay_device is None:
            print("Error: Relay device is not initialized. Please start the server first.")
            self.feedback_label.configure(
                text="Error: Relay device is not initialized. Start the server first.",
                text_color="red",
            )
            return

        # Toggle the relay state
        self.relay_states[index] = not self.relay_states[index]

        # Calculate the Modbus address for the relay
        relay_address = self.relay_device.start_address + index

        # Update the Modbus store with the new relay state
        self.relay_device.store.setValues(relay_address, [1 if self.relay_states[index] else 0])

        # Determine the register data based on the relay state
        register_data = 0x0001 if self.relay_states[index] else 0x0000

        # Send Modbus data to communicate the relay state
        self.relay_device.send_modbus_data(relay_address, register_data)

        # Update the relay button states in the GUI
        self.update_relay_buttons()

        # Save the updated relay states to the file
        self.save_relay_states()

    def update_relay_buttons(self):
        """Update the relay buttons in the GUI based on the current relay states."""
        for i, state in enumerate(self.relay_states):
            if i < len(self.relay_buttons):
                self.relay_buttons[i].configure(
                    text=f"Relay {i+1} {'ON' if state else 'OFF'}",
                    bg_color="green" if state else "red",
                    text_color="white" if state else "black",
                    font=("Arial", 12, "bold")  # Set font to bold
                )

    def manual_switch_press(self):
        if self.press_start_time is None:  # if not pressed
            self.press_start_time = time.time()
            self.switch_button.configure(text="Release to Switch")
            self.after(100, self.check_switch_duration)
        else:  # if already pressed
            duration = time.time() - self.press_start_time
            if (duration >= 5):  # If long pressed for more than 5 seconds
                self.enable_relay_timing()
                self.toggle_led_indicator(True)
            self.press_start_time = None
            self.switch_button.configure(text="Manual Switch")

    def check_switch_duration(self):
        if self.press_start_time is not None:
            self.after(100, self.check_switch_duration)

    def enable_relay_timing(self):
        self.toggle_led_indicator(True)
        time.sleep(600)  # Wait for 10 minutes
        self.toggle_led_indicator(False)

    def toggle_led_indicator(self, status):
        if status:
            self.led_indicator.configure(text="LED ON", text_color="green")
        else:
            self.led_indicator.configure(text="LED OFF", text_color="red")

    def handle_serial_communication(self):
        """Handle serial communication in a separate thread."""
        try:
            with serial.Serial(
                port=self.serial_port_menu.get(),
                baudrate=int(self.baudrate_entry.get()),
                parity=self.parity_entry.get(),
                stopbits=int(self.stopbits_entry.get()),
                bytesize=int(self.bytesize_entry.get()),
                timeout=1
            ) as ser:
                print("Serial communication thread started.")
                while True:
                    # Read data from the serial port
                    data = ser.read(1024)  # Adjust the buffer size as needed
                    if data:
                        print(f"Data received: {data.hex()}")
                        self.serial_queue.put(data)  # Add data to the queue for processing

                        # Example: Echo the data back
                        ser.write(data)
                        print(f"Data echoed back: {data.hex()}")
        except Exception as e:
            print(f"Error in serial communication thread: {e}")

    def process_serial_data(self):
        """Process data received from the serial thread."""
        while not self.serial_queue.empty():
            data = self.serial_queue.get()

            # Process the data (e.g., parse Modbus frame and update GUI)
            try:
                print(f"Processing data: {data.hex()}")

                # Parse the Modbus frame
                if len(data) >= 8:  # Minimum Modbus RTU frame length
                    slave_id = data[0]
                    function_code = data[1]
                    address = (data[2] << 8) | data[3]
                    value = (data[4] << 8) | data[5]
                    crc_received = (data[-2] | (data[-1] << 8))

                    # Validate CRC
                    crc_calculated = calculate_crc(data[:-2])  # Exclude the last two CRC bytes
                    if crc_calculated != crc_received:
                        print(f"CRC validation failed. Received: {crc_received:#06x}, Calculated: {crc_calculated:#06x}")
                        continue

                    # Handle Modbus logic based on function code
                    if function_code == 0x06:  # Write Single Register
                        relay_index = address - 0x0032  # Calculate relay index
                        if 0 <= relay_index < len(self.relay_states):
                            self.relay_states[relay_index] = bool(value)
                            print(f"Relay {relay_index + 1} set to {'ON' if value else 'OFF'}")
                            self.update_relay_buttons()
                            self.save_relay_states()
                        else:
                            print(f"Invalid relay address: {address:#06x}")

                    elif function_code == 0x03:  # Read Holding Registers
                        print(f"Read request received for address {address:#06x}, value: {value}")
                        # Handle read requests if needed (e.g., respond with register values)

                    else:
                        print(f"Unsupported function code: {function_code}")

                else:
                    print("Invalid Modbus frame received.")

            except Exception as e:
                print(f"Error processing serial data: {e}")

        # Schedule the next check
        self.after(100, self.process_serial_data)

    def save_relay_states(self):
        """Save the current relay states to the SwitchingActuatorData.json file."""
        try:
            data = {
                "type": self.relay_type,
                "quantity": self.relay_quantity,
                "data": [{"name": f"Relay#{i+1}", "value": state} for i, state in enumerate(self.relay_states)]
            }
            with open("SwitchingActuatorData.json", "w") as file:
                json.dump(data, file, indent=4)
            print("Relay states saved successfully.")
        except Exception as e:
            print(f"Error saving relay states: {e}")

    def load_switching_actuator_data(self):
        """Load relay states from SwitchingActuatorData.json and update the GUI."""
        try:
            with open("SwitchingActuatorData.json", "r") as file:
                data = json.load(file)
                print("Switching Actuator Data loaded successfully.")

                # Parse the data
                self.relay_type = data.get("type", "unknown")
                self.relay_quantity = data.get("quantity", 0)
                self.relay_data = data.get("data", [])

                # Initialize relay states based on the data
                self.relay_states = [relay.get("value", False) for relay in self.relay_data]

                # Update the GUI with the loaded states
                self.update_relay_buttons()
        except FileNotFoundError:
            print("SwitchingActuatorData.json file not found. Using default values.")
            self.relay_type = "switchingActuator"
            self.relay_quantity = 16
            self.relay_states = [False] * self.relay_quantity
        except Exception as e:
            print(f"Error loading SwitchingActuatorData.json: {e}")

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

class CallbackDataBlock(ModbusSparseDataBlock):
    def __init__(self, values, callback):
        super().__init__(values)
        self.callback = callback

    def getValues(self, address, count=1):
        """Handle read requests."""
        values = super().getValues(address, count)
        print(f"Read request received for address {address:#06x}, count: {count}")
        print(f"Returning values: {values}")
        return values

    def setValues(self, address, values):
        """Handle write requests."""
        super().setValues(address, values)
        self.callback(address, values)

def relay_callback(address, value, app):
    relay_index = address - 0x0032  # Calculate the relay index based on the address
    if 0 <= relay_index < 16:  # Ensure the address is within the relay range
        state = bool(value[0])  # Get the relay state (ON/OFF)
        app.relay_states[relay_index] = state
        app.update_relay_buttons()
        print(f"Relay {relay_index + 1} set to {'ON' if state else 'OFF'} via Modbus master")
    else:
        print(f"Invalid write request received for address {address:#06x} with value {value}")

def main():
    # Change the current working directory to so that opening relative paths/files will work
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)

    app = RelayApp()
    app.mainloop()

if __name__ == '__main__':
    main()