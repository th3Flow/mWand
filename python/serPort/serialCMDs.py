import serial
import time

def serialCMDs(cContainer):
    if cContainer.is_recording:
        if not cContainer.serial_port:
            try:
                serial_port = serial.Serial(cContainer.combobox.currentText(), 9600, timeout=1)
                cContainer.start_time = time.time()
            except serial.SerialException as e:
                print(f"Error opening serial port: {e}")
                cContainer.is_recording = False

        if cContainer.serial_port and cContainer.serial_port.isOpen():
            try:
                data = serial_port.readline().decode('utf-8').rstrip()
                if data:
                    with open(cContainer.filename_combobox.currentText(), "a") as file:
                        file.write(data + '\n')
                    print(data)
                    cContainer.start_time = time.time()  # Reset the timer after receiving data
                elif time.time() - cContainer.start_time > cContainer.TIMEOUT_SECONDS:
                    raise serial.SerialTimeoutException("Serial port read timeout.")
            except (serial.SerialTimeoutException, Exception) as e:
                print(f"Error during recording: {e}")
                cContainer.win.is_recording = False
                cContainer.serial_port.close()
                cContainer.serial_port = None

    else:
        if cContainer.serial_port and cContainer.serial_port.isOpen():
            cContainer.serial_port.close()
            cContainer.serial_port = None

    return cContainer