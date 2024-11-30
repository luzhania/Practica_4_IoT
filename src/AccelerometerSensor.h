#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

#pragma once

#define DEBOUNCE_TIME 1000

class AccelerometerSensor
{
private:
    Adafruit_MPU6050 accelerometer;
    unsigned int stepCount = 0;
    unsigned int temperature = 0;
    float threshold = 18;
    unsigned long lastStepTime = 0;
    float offsetX = 0, offsetY = 0;

    void calculateOffsets()
    {
        const int numReadings = 100;
        float sumX = 0, sumY = 0, sumZ = 0;

        Serial.println("Calculating offsets...");

        for (int i = 0; i < numReadings; i++)
        {
            sensors_event_t a, g, temp;
            accelerometer.getEvent(&a, &g, &temp);

            sumX += a.acceleration.x;
            sumY += a.acceleration.y;

            delay(10);
        }

        offsetX = sumX / numReadings;
        offsetY = sumY / numReadings;

        Serial.print("Offset X: ");
        Serial.println(offsetX);
        Serial.print("Offset Y: ");
        Serial.println(offsetY);
    }

public:
    void init()
    {
        while (!accelerometer.begin())
        {
            Serial.println("Failed to find MPU6050 chip");
            delay(10);
        }

        Serial.println("MPU6050 Found!");
        accelerometer.setAccelerometerRange(MPU6050_RANGE_8_G);
        accelerometer.setGyroRange(MPU6050_RANGE_500_DEG);
        accelerometer.setFilterBandwidth(MPU6050_BAND_5_HZ);
        calculateOffsets();
    }

    void updateSteps()
    {
        sensors_event_t a, g, temp;
        accelerometer.getEvent(&a, &g, &temp);
        float magnitude = sqrt(
            (a.acceleration.x - offsetX) * (a.acceleration.x - offsetX) +
            (a.acceleration.y - offsetY) * (a.acceleration.y - offsetY) +
            a.acceleration.z * a.acceleration.z);
        // Serial.print('magnitude ');
        // Serial.println(magnitude);

        if (magnitude > threshold)
        {
            unsigned long currentTime = millis();
            if (currentTime - lastStepTime > DEBOUNCE_TIME)
            {
                stepCount++;
                lastStepTime = currentTime;
                Serial.print("Step detected!");
            }
        }
    }

    unsigned int getStepCount()
    {
        return stepCount;
    }

    unsigned int getTemperature()
    {
        sensors_event_t a, g, temp;
        accelerometer.getEvent(&a, &g, &temp);
        return temp.temperature - 5;
    }

    unsigned int getAcceloremeterMagnitude()
    {
        sensors_event_t a, g, temp;
        accelerometer.getEvent(&a, &g, &temp);
        return sqrt(
            (a.acceleration.x - offsetX) * (a.acceleration.x - offsetX) +
            (a.acceleration.y - offsetY) * (a.acceleration.y - offsetY) +
            a.acceleration.z * a.acceleration.z);
    }
};