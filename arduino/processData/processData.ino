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
#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>

#include "model.h"
#include "model_params.h"

// IMU SETTINGS //
const float ACCEL_SCALE_FACTOR = 9.81;/* Your accelerometer scale factor */
const float GRAVITY = 9.81;
const int SAMPLE_RATE = 119; // Sample rate in Hz

const float accelerationThreshold = 3; // threshold of significant in m/s²
const int numSamples = 119;
// +++++++++++++++++++++++ //

// +++++++ PROCESS IMU GLOBALS +++++ //
float lastAx = 0, lastAy = 0, lastAz = 0;
float vx = 0, vy = 0, vz = 0;
float ax = 0, ay = 0, az = 0;
float x = 0, y = 0, z = 0;
const float GYRO_THRESHOLD = 0.02;
const float ACC_Treshold = 0.0; //* ACCEL_SCALE_FACTOR;

const int NUMDATA = 9;
float   data[12];
float   dataTf[NUMDATA];
int8_t  int8_data[NUMDATA];

int samplesRead = 0;

enum State 
{
    IDLE,
    GATHER
    // Add other states as needed
};

State currentState = IDLE;
// +++++++++++++++++++++++++ //

// +++++ CALIBRATION GLOBALS ++++++ //
// Variables to store the sum of the sensor readings
float accelSumX = 0, accelSumY = 0, accelSumZ = 0;
float gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;

// Variables to store the calculated bias
float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
const int calibrationSamples = 300;

// +++++++++++++++++++++++++ //

unsigned long currentTime = millis();
unsigned long previousTime = currentTime; // Initialize previousTime with the current time
float deltaTime; // Time difference in seconds

/* ++++++ FUNCTIONS +++++ */
void processIMU(float *data, float deltaTime);
float lowPassFilter(float currentVal, float previousVal, float alpha);
float fastAtan2(float y, float x);
float normalize_range(float val, float minR, float maxR);
/* ++++++++++++++++++++++++ */

// +++++ TENSORFLOW +++++ //
tflite::AllOpsResolver tflOpsResolver;

const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;

// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
constexpr int tensorArenaSize = 8 * 1024;
byte tensorArena[tensorArenaSize] __attribute__((aligned(16)));

// array to map gesture index to a name
const char* GESTURES[] = 
{
  "Alohomora",
  "Arresto M.",
  "Avada Kedavra",
  "Locomotor", 
  "Revelio"
};

#define NUM_GESTURES (sizeof(GESTURES) / sizeof(GESTURES[0]))

// +++++++++++++++++++++++++
void setup() 
{
    Serial.begin(9600);
    while (!Serial);

    if (!IMU.begin()) 
    {
      Serial.println("Failed to initialize IMU!");
      while (1);
    }
  
    /* ++++ IF ACTIVATED, the following function calculates the bias of the IMU */
    // calibrateSensors();

    // get the TFL representation of the model byte array
    tflModel = tflite::GetModel(model);
    if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
      Serial.println("Model schema mismatch!");
      while (1);
    }

    // Create an interpreter to run the model
    tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, tensorArenaSize);

    // Allocate memory for the model's input and output tensors
    tflInterpreter->AllocateTensors();

    // Get pointers for the model's input and output tensors
    tflInputTensor = tflInterpreter->input(0);
    tflOutputTensor = tflInterpreter->output(0);
    Serial.println("TensorFlow SET");
}

void loop() 
{
    float aSum;

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) 
    {
        deltaTime = 1.0f /SAMPLE_RATE; // Convert milliseconds to seconds

        processIMU(data, deltaTime);    
        aSum = fabs(data[0]) + fabs(data[1]) + fabs(data[2]);

        /*data[0] = normalize_range(data[0], -4, 4);
        data[1] = normalize_range(data[1], -4, 4);
        data[2] = normalize_range(data[2], -4, 4);

        data[3] = normalize_range(data[3], -2000, 2000);
        data[4] = normalize_range(data[4], -2000, 2000);
        data[5] = normalize_range(data[5], -2000, 2000);

        data[6] = normalize_range(data[6], 0, 360);
        data[7] = normalize_range(data[7], 0, 360);
        data[8] = normalize_range(data[8], 0, 360);

        data[9] = normalize_range(data[9], -0.25, 0.25);
        data[10] = normalize_range(data[10], -0.25, 0.25);
        data[11] = normalize_range(data[11], -0.25, 0.25);*/

        dataTf[0] = normalize_range(data[0], -4, 4);
        dataTf[1] = normalize_range(data[1], -4, 4);
        dataTf[2] = normalize_range(data[2], -4, 4);

        dataTf[3] = normalize_range(data[6], 0, 360);
        dataTf[4] = normalize_range(data[7], 0, 360);
        dataTf[5] = normalize_range(data[8], 0, 360);

        dataTf[6] = normalize_range(data[9], -0.25, 0.25);
        dataTf[7] = normalize_range(data[10], -0.25, 0.25);
        dataTf[8] = normalize_range(data[11], -0.25, 0.25);

        // CONVERSION
        for (int n = 0; n < NUMDATA; n++) 
        {
            int8_data[n] = static_cast<int8_t>((dataTf[n] / input_scale) + input_zero_point);
        }

        switch(currentState)
        {
          case IDLE:

            if (aSum >= accelerationThreshold) 
            {
                currentState = GATHER;             
                for (int i = 0; i < NUMDATA; i++)
                {
                    tflInputTensor->data.int8[samplesRead * NUMDATA + i] = int8_data[i];
                }          

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

            for (int i = 0; i < NUMDATA; i++)
            {
                tflInputTensor->data.int8[samplesRead * NUMDATA + i] = int8_data[i];
            }          

            samplesRead++;

            if (samplesRead >= numSamples) 
            {
                TfLiteStatus invokeStatus = tflInterpreter->Invoke();
                if (invokeStatus != kTfLiteOk) 
                {
                    Serial.println("Invoke failed!");
                    while (1);
                    return;
                }

                for (int i = 0; i < NUM_GESTURES; i++) 
                {
                    int8_t score = tflOutputTensor->data.int8[i];
                    Serial.print(GESTURES[i]);
                    Serial.print(": ");
                    Serial.print((score - output_zero_point) * output_scale * 100.0f, 2);
                    Serial.println(" %");
                }
                Serial.println();

                samplesRead = 0;
                lastAx = 0, lastAy = 0, lastAz = 0;
                vx = 0, vy = 0, vz = 0;
                x = 0, y = 0, z = 0;
                currentState = IDLE;
            }
            break;
        }
    }
}

void processIMU(float *data, float deltaTime) 
{
    IMU.readAcceleration(data[0], data[1], data[2]);
    IMU.readGyroscope(data[3], data[4], data[5]);

    /*data[0] -= accelBiasX;
    data[1] -= accelBiasY;
    data[2] -= accelBiasZ;
    data[3] -= gyroBiasX;
    data[4] -= gyroBiasY;
    data[5] -= gyroBiasZ;*/

    /*data[0] = data[0] * ACCEL_SCALE_FACTOR;
    data[1] = data[1] * ACCEL_SCALE_FACTOR;
    data[2] = data[2] * ACCEL_SCALE_FACTOR;*/

    // TRY TO denoise accelerometer data
    ax = lowPassFilter(data[0], lastAx, 0.5);
    ay = lowPassFilter(data[1], lastAy, 0.5);
    az = lowPassFilter(data[2], lastAz, 0.5);

    data[0] = ax;
    data[1] = ay;
    data[2] = az;

    lastAx = ax;
    lastAy = ay;
    lastAz = az;

    // Integrate twice and calculate alongation
    vx = ax * GRAVITY * deltaTime;
    vy = ay * GRAVITY * deltaTime;
    vz = az * GRAVITY * deltaTime;

    x += vx * deltaTime;
    y += vy * deltaTime;
    z += vz * deltaTime;

    // allongation in m
    data[9] = x;
    data[10] = y;
    data[11] = z;

    // TRY TO correct the gyro drift for x, and y 
    float rollAcc = fastAtan2(data[1]*GRAVITY,data[2]*GRAVITY) * (180 / M_PI); 
    data[6] = (data[3] * 0.8f) * deltaTime + rollAcc * 0.2f; 
    // Complementary filter roll 
    float pitchAcc = fastAtan2(data[0]*GRAVITY,data[2]*GRAVITY) * (180 / M_PI); 
    data[7] = (data[4] * 0.8f) * deltaTime + pitchAcc * 0.2f;

    if (fabs(data[5]) < GYRO_THRESHOLD) 
    {
        data[8] = 0;
    }
    else
    {
        data[8] = data[5] * deltaTime;
    }
}

float lowPassFilter(float currentVal, float previousVal, float alpha) 
{
    if (fabs(currentVal) < ACC_Treshold)
    {
      currentVal = 0;
    }
    return alpha * currentVal + (1 - alpha) * previousVal;
}

float diffFilter(float currentVal, float previousVal) 
{
    return currentVal - previousVal;
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

/* SENSOR CALIBRATION */
void calibrateSensors() 
{
    Serial.println("Put your Sensor into START Position...");
    for (int i = 4; i > 0; i--)
    {
        Serial.println(i);
        delay(1000);
    }
  
    Serial.println("Calibrating sensors...");
    float accelSumX = 0, accelSumY = 0, accelSumZ = 0;
    float gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
    float accelBiasX, accelBiasY, accelBiasZ;
    float gyroBiasX, gyroBiasY, gyroBiasZ;

    for (int i = 0; i < calibrationSamples; i++) 
    {
        float ax, ay, az;
        float gx, gy, gz;

        // Make sure you have fresh data before reading the sensors
        while (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable());

        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);

        accelSumX += ax;
        accelSumY += ay;
        accelSumZ += az;

        gyroSumX += gx;
        gyroSumY += gy;
        gyroSumZ += gz;

        delay(5); // Delay for a bit to not overload the I2C bus
    }
    
    // Calculate the bias for each sensor
    accelBiasX = accelSumX / calibrationSamples;
    accelBiasY = accelSumY / calibrationSamples;
    accelBiasZ = accelSumZ / calibrationSamples;

    gyroBiasX = gyroSumX / calibrationSamples;
    gyroBiasY = gyroSumY / calibrationSamples;
    gyroBiasZ = gyroSumZ / calibrationSamples;

    Serial.println("Done BIAS Calculation...");
    Serial.print("Accel Bias X: "); Serial.println(accelBiasX);
    Serial.print("Accel Bias Y: "); Serial.println(accelBiasY);
    Serial.print("Accel Bias Z: "); Serial.println(accelBiasZ);

    Serial.print("Gyro Bias X: "); Serial.println(gyroBiasX);
    Serial.print("Gyro Bias Y: "); Serial.println(gyroBiasY);
    Serial.print("Gyro Bias Z: "); Serial.println(gyroBiasZ);
}

float normalize_range(float val, float minR, float maxR)
{

    return (val - minR) / (maxR - minR);

}