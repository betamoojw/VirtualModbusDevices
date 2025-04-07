#!/usr/bin/python3

import os
import time
import threading
import customtkinter as ctk
import serial.tools.list_ports  # Import to list available serial ports
from pymodbus.server import StartSerialServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSparseDataBlock, ModbusSlaveContext, ModbusServerContext  # Add this import

# Configuration for Modbus
class RelayDevice:
    def __init__(self, baudrate=9600, parity='N', stopbits=1, bytesize=8):
        self.slave_id = 1
        self.store = ModbusSparseDataBlock({
            0: 0  # Relay status
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
        self.geometry("800x600")  # Adjusted window size for side-by-side layout

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

        ctk.CTkLabel(self.config_frame, text="Serial Port:").pack(pady=2)
        self.serial_ports = self.get_serial_ports()
        self.serial_port_menu = ctk.CTkOptionMenu(self.config_frame, values=self.serial_ports)
        self.serial_port_menu.pack(pady=5)

        ctk.CTkLabel(self.config_frame, text="Baud Rate:").pack(pady=2)
        self.baudrate_entry = ctk.CTkEntry(self.config_frame, placeholder_text="9600")
        self.baudrate_entry.pack(pady=5)

        ctk.CTkLabel(self.config_frame, text="Parity:").pack(pady=2)
        self.parity_entry = ctk.CTkEntry(self.config_frame, placeholder_text="N (None)")
        self.parity_entry.pack(pady=5)

        ctk.CTkLabel(self.config_frame, text="Stop Bits:").pack(pady=2)
        self.stopbits_entry = ctk.CTkEntry(self.config_frame, placeholder_text="1")
        self.stopbits_entry.pack(pady=5)

        ctk.CTkLabel(self.config_frame, text="Byte Size:").pack(pady=2)
        self.bytesize_entry = ctk.CTkEntry(self.config_frame, placeholder_text="8")
        self.bytesize_entry.pack(pady=5)

        self.start_button = ctk.CTkButton(self.config_frame, text="Start Server", command=self.start_server)
        self.start_button.pack(pady=10)

        self.update_serial_button = ctk.CTkButton(self.config_frame, text="Update Serial Config", command=self.update_serial_config)
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
        baudrate = self.baudrate_entry.get()
        baudrate = int(baudrate) if baudrate.isdigit() else 9600  # Use default 9600 if input is invalid
        parity = self.parity_entry.get()
        stopbits = self.stopbits_entry.get()
        stopbits = int(stopbits) if stopbits.isdigit() else 1  # Default to 1 if invalid
        bytesize = self.bytesize_entry.get()
        bytesize = int(bytesize) if bytesize.isdigit() else 8  # Default to 8 if invalid

        self.relay_device = RelayDevice(baudrate, parity, stopbits, bytesize)
        self.relay_device.serial_port = serial_port  # Set the selected serial port
        self.feedback_label.configure(text="Server started")

    def update_serial_config(self):
        if self.relay_device:
            try:
                serial_port = self.serial_port_menu.get()
                baudrate = self.baudrate_entry.get()
                baudrate = int(baudrate) if baudrate.isdigit() else 9600  # Use default 9600 if input is invalid
                parity = self.parity_entry.get()
                stopbits = self.stopbits_entry.get()
                stopbits = int(stopbits) if stopbits.isdigit() else 1  # Default to 1 if invalid
                bytesize = self.bytesize_entry.get()
                bytesize = int(bytesize) if bytesize.isdigit() else 8  # Default to 8 if invalid

                self.relay_device.configure_serial(baudrate, parity, stopbits, bytesize)
                self.relay_device.serial_port = serial_port  # Update the serial port
                self.feedback_label.configure(text="Serial configuration updated successfully.")
            except ValueError:
                self.feedback_label.configure(text="Invalid values. Please check your input.", text_color="red")

    def toggle_relay(self, index):
        # Toggle the relay state
        self.relay_states[index] = not self.relay_states[index]

        # Update the Modbus store with the new relay states
        coil_values = [1 if state else 0 for state in self.relay_states]
        self.relay_device.store.setValues(0, 0, coil_values)

        # Convert the coil values to a hexadecimal representation
        coil_binary = ''.join(str(bit) for bit in reversed(coil_values))  # Reverse for LSB first
        coil_hex = hex(int(coil_binary, 2))  # Convert binary string to hex

        # Print the Modbus code in hexadecimal
        print(f"Modbus code (hex): {coil_hex}")

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
            if duration >= 5:  # If long pressed for more than 5 seconds
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


def main():
    # Change the current working directory to so that opening relative paths/files will work
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)

    app = RelayApp()
    app.mainloop()

if __name__ == '__main__':
    main()