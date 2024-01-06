from PyQt5.QtWidgets import QApplication
from gui import MyWindow
import sys
import time
import serial
import matplotlib.pyplot as plt

def main():
    app = QApplication(sys.argv)
    win = MyWindow()
    win.show()

    serial_port = None
    start_time = None
    TIMEOUT_SECONDS = 4  # Set the timeout duration in seconds

    data_matrix = [[] for _ in range(119)]  # Initialize a 191x9 matrix
    row = 0  # Row counter
    doPlot = 0

    while True:
        app.processEvents()  # Process any GUI events

        if win.is_recording:
            if not serial_port:
                try:
                    serial_port = serial.Serial(win.combobox.currentText(), 9600, timeout=1)
                    start_time = time.time()
                except serial.SerialException as e:
                    print(f"Error opening serial port: {e}")
                    win.is_recording = False

            if serial_port and serial_port.isOpen():
                try:
                    data = serial_port.readline().decode('utf-8').rstrip()
                    if data:
                        parsed_data = data.split(',')  # Assuming comma-separated values
                        if len(parsed_data) == 9:
                            data_matrix[row] = parsed_data
                            row += 1
                            with open(win.filename_combobox.currentText(), "a") as file:
                                data_to_write = ",".join(str(item) for item in parsed_data[0:6])
                                file.write(data_to_write + '\n')
                        if row >= 119 or data == '':
                            row = 0
                            doPlot = 1
                            win.spellCnt += 1
                            win.labelCnt.setText(f"SpellCnt: {win.spellCnt}")
                            win.update()
                            with open(win.filename_combobox.currentText(), "a") as file:
                                file.write('\n')
                        print(data)
                        start_time = time.time()  # Reset the timer after receiving data
                    elif time.time() - start_time > TIMEOUT_SECONDS:
                        raise serial.SerialTimeoutException("Serial port read timeout.")
                except (serial.SerialTimeoutException, Exception) as e:
                    print(f"Error during recording: {e}")
                    win.is_recording = False
                    serial_port.close()
                    serial_port = None

        else:
            if serial_port and serial_port.isOpen():
                serial_port.close()
                serial_port = None

        time.sleep(0.001)

        if doPlot:
            plot_recDat(data_matrix)
            doPlot = 0

        if win.closer:
            break

    if serial_port and serial_port.isOpen():
        serial_port.close()

    sys.exit()

def plot_recDat(data_matrix):

    # Extract columns 8 and 9
    x = [float(row[7]) for row in data_matrix]  # Column 7
    z = [float(row[8]) for row in data_matrix]  # Column 9

    # Create a scatter plot
    plt.figure()
    plt.scatter(x, z)
    #plt.xlim([-0.5, 0.5])
    #plt.ylim([-0.5, 0.5])

    plt.xlabel('x (m)')
    plt.ylabel('z (m)')
    plt.title('Scatter Plot Recorded Data')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()

