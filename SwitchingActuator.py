#!/usr/bin/python3

import os
import time
import threading
import customtkinter as ctk
import serial.tools.list_ports  # Import to list available serial ports
import configparser  # Import configparser to handle config.ini
from pymodbus.server import StartSerialServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSparseDataBlock, ModbusSlaveContext, ModbusServerContext  # Add this import

# Configuration for Modbus
class RelayDevice:
    def __init__(self, baudrate=9600, parity='N', stopbits=1, bytesize=8):
        self.slave_id = 1
        self.start_address = 0x0032  # Starting address in hexadecimal
        self.store = ModbusSparseDataBlock({
            self.start_address + i: 0 for i in range(16)  # Initialize 16 relays with consecutive addresses
        })

        self.baudrate = baudrate
        self.parity = parity
        self.stopbits = stopbits
        self.bytesize = bytesize
        self.serial_port = "COM9"  # Default serial port

        self.start_rtu_server()

    def start_rtu_server(self):
        # Use ModbusSlaveContext instead of ModbusContext
        context = ModbusSlaveContext(
            di=None,
            co=self.store,
            hr=None,
            ir=None,
        )
        # Wrap the context in a ModbusServerContext
        server_context = ModbusServerContext(slaves=context, single=True)
        threading.Thread(target=self.run_rtu_server, args=(server_context,), daemon=True).start()

    def run_rtu_server(self, context):
        StartSerialServer(context, port=self.serial_port, baudrate=self.baudrate,
                          parity=self.parity, stopbits=self.stopbits, bytesize=self.bytesize)

    def configure_serial(self, baudrate, parity, stopbits, bytesize):
        self.baudrate = baudrate
        self.parity = parity
        self.stopbits = stopbits
        self.bytesize = bytesize
        self.restart_rtu_server()

    def restart_rtu_server(self):
        print("Restarting RTU server with new parameters")
        self.start_rtu_server()


# GUI Application
class RelayApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Virtual Modbus Slave")
        self.geometry("860x680")  # Adjusted window size for side-by-side layout

        # Load configuration from config.ini
        self.config = configparser.ConfigParser()
        self.config.read("config.ini")
        self.serial_config = self.config["SerialConfig"]

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
        for i in range(16):
            row = i // 2  # Determine the row (0-7)
            col = i % 2   # Determine the column (0-1)
            btn = ctk.CTkButton(self.relay_frame, text=f"Relay {i+1}", command=lambda i=i: self.toggle_relay(i))
            btn.grid(row=row + 1, column=col, padx=5, pady=5, sticky="nsew")
            self.relay_buttons.append(btn)

        # Configure grid weights for relay buttons
        for r in range(8):  # 8 rows
            self.relay_frame.grid_rowconfigure(r + 1, weight=1)
        for c in range(2):  # 2 columns
            self.relay_frame.grid_columnconfigure(c, weight=1)

        self.switch_button = ctk.CTkButton(self.relay_frame, text="Manual Switch", command=self.manual_switch_press)
        self.switch_button.grid(row=9, column=0, columnspan=2, pady=10)

        self.led_indicator = ctk.CTkLabel(self.relay_frame, text="LED OFF", text_color="red")
        self.led_indicator.grid(row=10, column=0, columnspan=2, pady=10)

        # Long press variables
        self.press_start_time = None

    def get_serial_ports(self):
        """Retrieve a list of available serial ports."""
        ports = serial.tools.list_ports.comports()
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

        self.relay_device = RelayDevice(baudrate, parity, stopbits, bytesize)
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
        # Toggle the relay state
        self.relay_states[index] = not self.relay_states[index]

        # Calculate the Modbus address for the relay
        relay_address = self.relay_device.start_address + index

        # Update the Modbus store with the new relay state
        self.relay_device.store.setValues(3, relay_address, [1 if self.relay_states[index] else 0])  # 3 = Holding Register

        # Determine the register data based on the relay state
        register_data = 0x0001 if self.relay_states[index] else 0x0000

        # Modbus frame components
        slave_id = self.relay_device.slave_id
        function_code = 0x06  # Function code for writing a single register
        address_high = (relay_address >> 8) & 0xFF
        address_low = relay_address & 0xFF
        data_high = (register_data >> 8) & 0xFF
        data_low = register_data & 0xFF

        # Construct the Modbus frame
        modbus_frame = bytes([slave_id, function_code, address_high, address_low, data_high, data_low])

        # Calculate CRC using the custom calculate_crc function
        crc = calculate_crc(modbus_frame)
        crc_lsb = crc & 0xFF  # Least significant byte
        crc_msb = (crc >> 8) & 0xFF  # Most significant byte

        # Print the Modbus address, state, slave ID, function code, and CRC in hexadecimal
        print(f"Relay {index + 1} at address {relay_address:#06x} set to {'ON' if self.relay_states[index] else 'OFF'}")
        print(f"Slave ID: {slave_id:#04x}, Function Code: {function_code:#04x}")
        print(f"Address: {relay_address:#06x}, Data: {register_data:#06x}")
        print(f"CRC: {crc_msb:#04x} {crc_lsb:#04x}")  # Print CRC
        print(f"CRC (swapped): {crc_lsb:#04x} {crc_msb:#04x}")  # Print swapped CRC

        # Update the relay button states in the GUI
        self.update_relay_buttons()

    def update_relay_buttons(self):
        for i, state in enumerate(self.relay_states):
            self.relay_buttons[i].configure(bg_color="green" if state else "red", text=f"Relay {i+1} {'ON' if state else 'OFF'}")

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


def main():
    # Change the current working directory to so that opening relative paths/files will work
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)

    app = RelayApp()
    app.mainloop()

if __name__ == '__main__':
    main()