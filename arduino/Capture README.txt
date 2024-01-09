############################################
#
#
#	README ARDUINO Capture
#	FLORIAN MAYER
#	florian.mayer@fh-joanneum.at
#
############################################


IMU Capture for Gesture Recognition

Overview
This Arduino script, was modified and enhanced, captures and processes IMU (Inertial Measurement Unit) data for gesture recognition. 
It's specifically designed for the Arduino Nano 33 BLE boards. The script reads acceleration and gyroscope data, applies denoising, corrects gyro drift, and calculates position changes.

Key Features

IMU Data Processing: Acceleration and gyroscope data are read and processed for gesture detection.
Denoising and Drift Correction: Incorporates techniques to denoise accelerometer data and correct gyro drift for improved accuracy.
Customizable Sample Rate: Set to sample at 119 Hz, but can be adjusted as needed.
Significant Motion Detection: Captures data when significant motion is detected, based on a predefined acceleration threshold.
Output Data Structure: Outputs processed IMU data in a structured format for further analysis or gesture recognition.

Modifications
22.12.23: Added denoising for accelerometer data and drift removal for gyroscope data.
25.12.23: Implemented a faster atan2 function for more efficient processing.

Setup and Usage
Hardware Requirements: Arduino Nano 33 BLE or Arduino Nano 33 BLE Sense board.
Software Requirements: Arduino IDE with the necessary board and library configurations.
Script Installation: Upload the script to the Arduino board using the Arduino IDE.
Data Capture: Data is captured and printed to the Serial Monitor or Serial Plotter when significant motion is detected.
Customization: Adjust the SAMPLE_RATE, accelerationThreshold, and other constants as needed for your specific application.

Notes
Ensure the Arduino LSM9DS1 library is installed and properly configured in your Arduino IDE.
The script is designed for real-time data processing and may need to be adapted for different IMU models or applications.