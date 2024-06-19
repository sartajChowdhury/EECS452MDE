#This Python script is responsible for reading in the Serial data that ECG_Final.ino is outputting and plotting the data in real-time. 
#We developed and tested this code using Visual Studio Code. After running this script, it will wait to receive the "START" message from ECG_Final.ino before it starts plotting the ECG data in real time. 
#This script makes use of deques, blipping and the FuncAnimation library #to effectively plot in real-time at 300 Hz.

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import time
from threading import Thread

# Variables
WINDOW_SIZE = 600  # Width of the window that will be displayed in samples
SAMPLING_FREQUENCY = 300  # Sampling frequency for ECG signal (Hz)
WINDOW_SIZE_SECONDS = WINDOW_SIZE / SAMPLING_FREQUENCY  # Seconds in the window

# Axis Bounds
Y_axis_lower = -100
Y_axis_upper = 500

# Serial port settings
PORT = '/dev/cu.usbserial-1420'  # Change this to your serial port
BAUD_RATE = 115200

# Start message
START_MESSAGE = b"START"
ACK_MESSAGE = b"ACK"

# Global variables for deque
x_values = deque(maxlen=WINDOW_SIZE)
y_values = deque(maxlen=WINDOW_SIZE)
start_time = time.time()

def update_plot(frame, line, ax):
    if len(x_values) > 0:
        line.set_data(list(x_values), list(y_values))
        # Update x-axis to move with time, displaying a window that scrolls after initial period
        current_time = time.time() - start_time
        ax.set_xlim(max(0, current_time - WINDOW_SIZE_SECONDS), max(WINDOW_SIZE_SECONDS, current_time))
    return line,

def read_from_port(ser):
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().strip().decode()
            try:
                current_time = time.time()
                y = float(data)
                x_values.append(current_time - start_time)
                y_values.append(y)
                # print(f"Data: {y}, Time: {current_time - start_time}")  # Debug print
            except ValueError:
                print("Failed to convert data to float")  # Error handling

if __name__ == "__main__":
    ser = serial.Serial(PORT, BAUD_RATE, timeout=None)
    print("Waiting for start message...")
    while True:
        message = ser.readline().strip()
        if message == START_MESSAGE:
            ser.write(ACK_MESSAGE)
            print("Start message received and acknowledged")
            break

    fig, ax = plt.subplots()
    ax.set_title('Live Plot')
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Voltage (mV)')
    ax.set_ylim(Y_axis_lower, Y_axis_upper)
    line, = ax.plot([], [], 'b-')

    ani = FuncAnimation(fig, update_plot, fargs=(line, ax), interval=50, blit=True)

    # Start a thread to read data from the port
    thread = Thread(target=read_from_port, args=(ser,))
    thread.daemon = True
    thread.start()

    plt.show()
