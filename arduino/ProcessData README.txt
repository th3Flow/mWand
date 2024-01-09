############################################
#
#
#	README ARDUINO Process Data
#	FLORIAN MAYER
#	florian.mayer@fh-joanneum.at
#
############################################

Gesture Recognition with Arduino Nano 33 BLE

Overview
This script, designed for the Arduino Nano 33 BLE boards, captures and processes data from the onboard IMU (Inertial Measurement Unit) for gesture recognition. The program reads acceleration and gyroscope data, applies noise reduction, corrects gyro drift, and calculates position. The script then uses TensorFlow Lite for Microcontrollers to classify the captured gesture into one of several predefined categories.

Features
Real-Time IMU Data Processing: Reads acceleration and gyroscope data for gesture recognition.
Noise Reduction and Drift Correction: Advanced techniques for improving data accuracy.
Gesture Recognition: Utilizes TensorFlow Lite for Microcontrollers to classify gestures.
Customizable Settings: Sample rate and other parameters can be adjusted as needed.

Setup
Hardware: Arduino Nano 33 BLE or BLE Sense board.
Software: Arduino IDE with the necessary libraries.
Installation: Upload the script to your Arduino board using the Arduino IDE.

Usage
The script automatically starts reading data when significant motion is detected.
Processed data is printed to the Serial Monitor, and gesture classification results are displayed.
You can visualize the IMU data using the Serial Plotter.

Calibration
Calibrate the sensors for accurate readings.
Follow the on-screen instructions in the Serial Monitor to calibrate.

TensorFlow Lite Integration
The script uses a TensorFlow Lite model for gesture classification.
The model is included as a C array in 'model.h'.
Gesture categories and TensorFlow Lite settings are configurable.