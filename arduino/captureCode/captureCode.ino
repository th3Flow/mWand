/*
  IMU Capture

  This example uses the on-board IMU to start reading acceleration and gyroscope
  data from on-board IMU and prints it to the Serial Monitor for one second
  when the significant motion is detected.

  You can also use the Serial Plotter to graph the data.

  The circuit:
  - Arduino Nano 33 BLE or Arduino Nano 33 BLE Sense board.

  Created by Don Coleman, Sandeep Mistry
  Modified by Dominic Pajak, Sandeep Mistry

  This example code is in the public domain.
*/

/* MODIFIED BY Mayer Florian 
  ++++++++++++++++++++++++++++++++++++++++++++++++++
  22.12.23 denoise ACCEL, remove drift from GYRO
  25.12.23 speed up atan2 with FASTatan2
  ++++++++++++++++++++++++++++++++++++++++++++++++++
*/

#include <Arduino_LSM9DS1.h>

const float ACCEL_SCALE_FACTOR = 9.81;/* Your accelerometer scale factor */
const float GRAVITY = 9.81;
const int SAMPLE_RATE = 119; // Sample rate in Hz

const float accelerationThreshold = 3; // threshold of significant in m/s²
const int numSamples = 119;

float lastAx = 0, lastAy = 0, lastAz = 0;
float vx = 0, vy = 0, vz = 0;
float ax = 0, ay = 0, az = 0;
float x = 0, y = 0, z = 0;
// Variables to store the sum of the sensor readings
float accelSumX = 0, accelSumY = 0, accelSumZ = 0;
float gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;

// Variables to store the calculated bias
float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
const int calibrationSamples = 300;
int samplesRead = 0;

const float GYRO_THRESHOLD = 0.02;
const float ACC_Treshold = 0.25 * ACCEL_SCALE_FACTOR;
char buffer[100];
float data[9];

void gatherIMU(float *data, float deltaTime);

enum State 
{
    IDLE,
    GATHER
    // Add other states as needed
};

State currentState = IDLE;

unsigned long currentTime = millis();
unsigned long previousTime = currentTime; // Initialize previousTime with the current time
float deltaTime; // Time difference in seconds

void setup() 
{
    Serial.begin(9600);
    while (!Serial);

    if (!IMU.begin()) 
    {
      Serial.println("Failed to initialize IMU!");
      while (1);
    }
  
}

void loop() 
{
    float aSum;

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) 
    {
        currentTime = millis();
        deltaTime = (currentTime - previousTime) / 1000.0; // Convert milliseconds to seconds

        gatherIMU(data, deltaTime);    
        previousTime = currentTime;

        switch(currentState)
        {
          case IDLE:
            aSum = fabs(data[0]) + fabs(data[1]) + fabs(data[2]);

            if (aSum >= accelerationThreshold) 
            {
                currentState = GATHER;
                sprintf(buffer, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
                Serial.println(buffer);
                samplesRead++;
            }
            else
            {
                samplesRead = 0;
                lastAx = 0, lastAy = 0, lastAz = 0;
                vx = 0, vy = 0, vz = 0;
                x = 0, y = 0, z = 0;
            }
            break;

          case GATHER:

            sprintf(buffer, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
            Serial.println(buffer);
            samplesRead++;

            if (samplesRead >= numSamples) 
            {
                samplesRead = 0;
                lastAx = 0, lastAy = 0, lastAz = 0;
                vx = 0, vy = 0, vz = 0;
                x = 0, y = 0, z = 0;
                Serial.println();
                currentState = IDLE;
            }
            break;
        }
    }
}

void gatherIMU(float *data, float deltaTime) 
{
    IMU.readAcceleration(data[0], data[1], data[2]);
    IMU.readGyroscope(data[3], data[4], data[5]);
  /*
    data[0] -= accelBiasX;
    data[1] -= accelBiasY;
    data[2] -= accelBiasZ;
    data[3] -= gyroBiasX;
    data[4] -= gyroBiasY;
    data[5] -= gyroBiasZ; */

/*
    data[0] = data[0] * ACCEL_SCALE_FACTOR;
    data[1] = data[1] * ACCEL_SCALE_FACTOR;
    data[2] = data[2] * ACCEL_SCALE_FACTOR; */

    // TRY TO denoise accelerometer data
    ax = lowPassFilter(data[0] * ACCEL_SCALE_FACTOR, lastAx, 0.85);
    ay = lowPassFilter(data[1] * ACCEL_SCALE_FACTOR, lastAy, 0.85);
    az = lowPassFilter(data[2] * ACCEL_SCALE_FACTOR - GRAVITY, lastAz, 0.85);

    /*data[0] = data[0] * ACCEL_SCALE_FACTOR;
    data[1] = data[1] * ACCEL_SCALE_FACTOR;
    data[2] = data[2] * ACCEL_SCALE_FACTOR; */

    lastAx = ax;
    lastAy = ay;
    lastAz = az;

    // Integrate twice and calculate alongation
    vx = ax * deltaTime;
    vy = ay * deltaTime;
    vz = az * deltaTime;

    x += vx * deltaTime;
    y += vy * deltaTime;
    z += vz * deltaTime;

    // allongation in m
    data[6] = x;
    data[7] = y;
    data[8] = z;
}

float lowPassFilter(float currentVal, float previousVal, float alpha) 
{
    if (fabs(currentVal) < ACC_Treshold)
    {
      currentVal = 0;
    }
    return alpha * currentVal + (1 - alpha) * previousVal;
}

float fastAtan2(float y, float x) 
{
    const float n1 = 0.97239411f;
    const float n2 = -0.19194795f;
    float result = 0.0f;
    if (x != 0.0f) {
        const float atan = (float)M_PI / 4.0f * y / x;
        const float z = atan * atan;
        result = atan * ((n1 + n2 * z) / (1.0f + (n1 + n2) * z));
        if (x < 0.0f) {
            if (y < 0.0f) {
                return result - (float)M_PI;
            }
            return result + (float)M_PI;
        }
    } else if (y > 0.0f) {
        result = (float)M_PI / 2.0f;
    } else if (y < 0.0f) {
        result = -(float)M_PI / 2.0f;
    }
    return result;
}