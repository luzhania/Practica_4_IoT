#include <Arduino.h>
#include "MAX30100_PulseOximeter.h"

#pragma once


class PulseOximeterSensor
{
private:
    PulseOximeter pox;

    static void onBeatDetected()
    {
        Serial.println("Beat detected!");
    }

public:
    void init()
    {
        while (!pox.begin())
        {
            Serial.println("FAILED to initialize pulse oximeter!");
        }
        Serial.println("SUCCESS to initialize pulse oximeter!");
        pox.setIRLedCurrent(MAX30100_LED_CURR_46_8MA);
        pox.setOnBeatDetectedCallback(onBeatDetected);
    }

    void update()
    {
        pox.update();
    }

    float getHeartRate()
    {
        return pox.getHeartRate();
    }

    uint8_t getSpO2()
    {
        return pox.getSpO2();
    }
};
