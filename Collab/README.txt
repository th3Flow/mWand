############################################
#
#
#	README GOOGLE Collab
#	FLORIAN MAYER
#	florian.mayer@fh-joanneum.at
#
############################################

# Magic Wand Gesture Recognition Model Training

## Overview
This script trains a neural network model to recognize wand gestures from IMU sensor data, suitable for use in applications like gesture-controlled systems. It processes IMU data, normalizes it, creates a model, trains it, and evaluates its performance.

## Prerequisites
- Python 3.x
- TensorFlow (preferably the latest version)
- NumPy
- Matplotlib
- Scikit-learn
- Pandas

++++++++++++++++++++++++++++++++++++++++++++++
++++++++++++++++++++++++++++++++++++++++++++++

## Data Format
The input data should be in CSV format with IMU sensor readings. Each row represents a sample with the following columns:
- Accelerometer readings (ax, ay, az)
- Gyroscope readings (gx, gy, gz)

## Features
- Data preprocessing includes normalization and low-pass filtering.
- The neural network architecture is a fully connected deep learning model with regularization and dropout to prevent overfitting.
- The script includes functions to evaluate the model using a confusion matrix and calculate model accuracy.
- The trained model can be exported to TensorFlow Lite format for deployment on edge devices.

## Customization
To adjust the training process or model architecture, modify the constants and parameters at the beginning of the script.

## Output
The script will output the trained model files in TensorFlow Lite format and display a confusion matrix of the model's predictions.

## License
This project is licensed under the MIT License - see the LICENSE.md file for details.

## Acknowledgments
- Inspired by the magic of wand gestures in popular culture.
- Thanks to the TensorFlow team for providing a comprehensive machine learning library.
