# mWand

This project involved developing a machine learning model for gesture recognition based on movement data captured by an IMU (Inertial Measurement Unit). The data, consisting of accelerometer and gyroscope readings, was processed and normalized to feed into a neural network built using TensorFlow. The neural network, consisting of dense layers with dropout and regularization, was trained to classify different gestures, and its architecture was optimized for performance and efficiency. Once trained, the model was converted into a TensorFlow Lite format suitable for deployment on an Arduino board. The final stage involved implementing an Arduino sketch to process real-time IMU data, perform inference using the trained model, and recognize gestures, thereby bridging the gap between machine learning and embedded systems in a real-world application.