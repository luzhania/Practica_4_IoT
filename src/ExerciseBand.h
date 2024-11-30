#include "MQTTClient.h"

#pragma once


class ExerciseBand : public MQTTClient
{
protected:
    const char *UPDATE_TOPIC = "$aws/things/smartband_0001/shadow/update";
    const char *UPDATE_DELTA_TOPIC = "$aws/things/smartband_0001/shadow/update/delta";

    unsigned int pulse = 0;
    unsigned int bloodOxygen = 0;
    unsigned int steps = 0;
    unsigned int temperature = 0;
    String activityType = "Reposo";
    unsigned int currentState = -1;
    unsigned int lastStepsReported = 0;
    unsigned int minPulseAlert = 60;
    unsigned int maxPulseAlert = 200;
    String message = "";

    StaticJsonDocument<JSON_OBJECT_SIZE(512)> outputDoc;
    char outputBuffer[512];

public:
    ExerciseBand(const char *broker, int port) : MQTTClient(broker, port) {}

    void onMessageReceived(const String &topic, StaticJsonDocument<200> inputDoc) override
    {
        Serial.println("Message received from: " + topic);
        if (String(topic) == UPDATE_DELTA_TOPIC)
        {
            if (inputDoc["state"]["message"])
            {
                message = inputDoc["state"]["message"].as<String>();
                reportMessage();
            }
            if (inputDoc["state"]["data_requested"] == 1)
            {
                publishDataRequestAttended();
                updateDataInShadow();
            }
            if (inputDoc["state"]["min_pulse_alert"] > 0)
            {
                minPulseAlert = inputDoc["state"]["min_pulse_alert"];
                reportMinPulseParameter();
            }
            if (inputDoc["state"]["max_pulse_alert"] > 0)
            {
                maxPulseAlert = inputDoc["state"]["max_pulse_alert"];
                reportMaxPulseParameter();
            }
        }
    }

    void subscribeTopics() override
    {
        client.subscribe(UPDATE_DELTA_TOPIC);
        Serial.println("Suscribed to topic: " + String(UPDATE_DELTA_TOPIC));
    }

    void updateDataInShadow()
    {
        outputDoc.clear();
        outputDoc["state"]["reported"]["heart_rate"] = pulse;
        outputDoc["state"]["reported"]["SpO2"] = bloodOxygen;
        outputDoc["state"]["reported"]["activity_type"] = activityType;
        outputDoc["state"]["reported"]["environment_temperature"] = temperature;
        outputDoc["state"]["reported"]["steps"] = steps;

        serializeJson(outputDoc, outputBuffer);
        client.publish(UPDATE_TOPIC, outputBuffer);
    }

    void publishDataRequestAttended()
    {
        outputDoc.clear();
        outputDoc["state"]["desired"]["data_requested"] = 0;
        serializeJson(outputDoc, outputBuffer);
        client.publish(UPDATE_TOPIC, outputBuffer);
    }

    void reportMinPulseParameter()
    {
        outputDoc.clear();
        outputDoc["state"]["reported"]["min_pulse_alert"] = minPulseAlert;
        serializeJson(outputDoc, outputBuffer);
        client.publish(UPDATE_TOPIC, outputBuffer);
    }

    void reportMaxPulseParameter()
    {
        outputDoc.clear();
        outputDoc["state"]["reported"]["max_pulse_alert"] = maxPulseAlert;
        serializeJson(outputDoc, outputBuffer);
        client.publish(UPDATE_TOPIC, outputBuffer);
    }

    void reportMessage()
    {
        outputDoc.clear();
        outputDoc["state"]["reported"]["message"] = message.c_str();
        serializeJson(outputDoc, outputBuffer);
        client.publish(UPDATE_TOPIC, outputBuffer);
    }

    unsigned int getNewState()
    {
        if (pulse < minPulseAlert)
            return 0;
        if (pulse < maxPulseAlert)
            return 1;
        return 2;
    }

    String getNewActivityType(unsigned int accelerometerMagnitude)
    {
        const int stepThreshold = 18;
        const int reposePulseThreshold = 70;
        const int exercisePulseThreshold = 120;
        
        Serial.print("Accelerometer magnitude: ");
        Serial.println(accelerometerMagnitude);
        Serial.print("pulsr: ");
        Serial.println(pulse);
        if (accelerometerMagnitude < stepThreshold && pulse < reposePulseThreshold)
        {
            return "Reposo";
        }
        else if (accelerometerMagnitude >= stepThreshold && pulse < exercisePulseThreshold)
        {
            return "Movimiento";
        }
        else if (accelerometerMagnitude >= stepThreshold && pulse >= exercisePulseThreshold)
        {
            return "Ejercicio";
        }
        return "Reposo";
    }

    void reportData(unsigned int pulse, unsigned int pulseOx, unsigned int steps, unsigned int temperature, unsigned int accelerometerMagnitude)
    {
        this->pulse = pulse;
        this->bloodOxygen = pulseOx;
        this->steps = steps;
        this->temperature = temperature;
        if (getNewState() != currentState || steps - lastStepsReported > 200 || getNewActivityType(accelerometerMagnitude) != activityType)
        {

            currentState = getNewState();
            lastStepsReported = steps;
            activityType = getNewActivityType(accelerometerMagnitude);
            
            outputDoc.clear();
            outputDoc["state"]["reported"]["heart_rate_state"] = currentState;
            outputDoc["state"]["reported"]["heart_rate"] = pulse;
            outputDoc["state"]["reported"]["SpO2"] = bloodOxygen;
            outputDoc["state"]["reported"]["activity_type"] = activityType;
            outputDoc["state"]["reported"]["environment_temperature"] = temperature;
            outputDoc["state"]["reported"]["steps"] = steps;

            serializeJson(outputDoc, outputBuffer);
            client.publish(UPDATE_TOPIC, outputBuffer);
            Serial.print("New state published: ");
            Serial.println(currentState);
        }
    }

    String getMessage()
    {
        return message;
    }

    String getActivityType()
    {
        return activityType;
    }
};