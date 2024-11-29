#include <Arduino.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <WiFi.h>
#include "WiFiConnection.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "config.h"

#define REPORTING_PERIOD_MS 600

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

class AccelerometerSensor
{
private:
    Adafruit_MPU6050 accelerometer;
    unsigned int stepCount = 0;
    float threshold = 20;
    unsigned long lastStepTime = 0;
    unsigned long debounceTime = 300;
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

    void update_steps()
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
            if (currentTime - lastStepTime > debounceTime)
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
};

class LCDDisplay
{
private:
    LiquidCrystal_I2C lcd;

public:
    LCDDisplay() : lcd(0x27, 20, 4) {}

    void init()
    {
        lcd.init();
        lcd.backlight();
    }

    void printMessage(unsigned int pulse, unsigned int row, unsigned int col, String message)
    {
        lcd.clear();
        lcd.setCursor(col, row);
        lcd.print(message.c_str());
        lcd.setCursor(0, 3);
        String pulseMessage = String(pulse) + " lpm";
        lcd.print(pulseMessage);
    }
};

class MQTTClient
{
protected:
    WiFiClientSecure wiFiClient;
    PubSubClient client;

    void callback(char *topic, byte *payload, unsigned int length)
    {
        String message;
        for (unsigned int i = 0; i < length; i++)
        {
            message += (char)payload[i];
        }
        Serial.println("Message from topic " + String(topic) + ": " + message);
        StaticJsonDocument<200> inputDoc;
        DeserializationError error = deserializeJson(inputDoc, payload, length);
        if (!error)
        {
            onMessageReceived(topic, inputDoc);
        }
        else
        {
            Serial.println("Error deserializando el mensaje JSON");
        }
    }

    virtual void onMessageReceived(const String &topic, StaticJsonDocument<200> inputDoc) = 0;
    virtual void subscribeTopics() = 0;

public:
    MQTTClient(const char *broker, int port) : client(wiFiClient)
    {
        client.setServer(broker, port);
        client.setCallback([this](char *topic, byte *payload, unsigned int length)
                           { this->callback(topic, payload, length); });
    }

    void setCertificates(const char *rootCA, const char *cert, const char *key)
    {
        wiFiClient.setCACert(rootCA);
        wiFiClient.setCertificate(cert);
        wiFiClient.setPrivateKey(key);
    }

    void connectMQTT()
    {
        while (!client.connected())
        {
            Serial.print("Trying MQTT conection...");
            if (client.connect("CLIENT_ID"))
            {
                Serial.println("Conected");
                subscribeTopics();
            }
            else
            {
                Serial.print("error, rc=");
                Serial.print(client.state());
                Serial.println(" trying again in 5 seconds");
                delay(5000);
            }
        }
    }

    void loop()
    {
        client.loop();
    }

    bool isConnected()
    {
        return client.connected();
    }
};

class ExerciseBand : public MQTTClient
{
protected:
    const char *UPDATE_TOPIC = "$aws/things/exerciseband/shadow/update";
    const char *UPDATE_DELTA_TOPIC = "$aws/things/exerciseband/shadow/update/delta";

    unsigned int pulse = 0;
    unsigned int currentState = 0;
    unsigned int minPulseAlert = 60;
    unsigned int maxPulseAlert = 200;
    String message = "";

    StaticJsonDocument<JSON_OBJECT_SIZE(64)> outputDoc;
    char outputBuffer[128];

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
                publishPulseRequestAttended();
                updatePulseInShadow();
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

    void updatePulseInShadow()
    {
        outputDoc.clear();
        outputDoc["state"]["reported"]["heart_rate"] = pulse;
        outputDoc["state"]["reported"]["SpO2"] = 98; //////
        outputDoc["state"]["reported"]["activity_type"] = 'Actividad'; //////
        outputDoc["state"]["reported"]["enviroment_temperature"] = 25; //////
        outputDoc["state"]["reported"]["steps"] = 50;     //////

        serializeJson(outputDoc, outputBuffer);
        client.publish(UPDATE_TOPIC, outputBuffer);
    }

    void publishPulseRequestAttended()
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

    void reportStateIfChanged(unsigned int pulse)
    {
        this->pulse = pulse;
        if (getNewState() != currentState)
        {
            currentState = getNewState();
            outputDoc.clear();
            outputDoc["state"]["reported"]["heart_rate_state"] = currentState;
            serializeJson(outputDoc, outputBuffer);
            client.publish(UPDATE_TOPIC, outputBuffer);
            Serial.print("Publicado nuevo estado: ");
            Serial.println(currentState);
        }
    }

    String getMessage()
    {
        return message;
    }
};

WiFiConnection wifi(WIFI_SSID, WIFI_PASS);
ExerciseBand exerciseBand(MQTT_BROKER, MQTT_PORT);

LCDDisplay lcd;
AccelerometerSensor accelerometer;

// Time at which the last beat occurred
uint32_t tsLastReport = 0;

PulseOximeterSensor pulseOximeter;

void setup()
{
    Serial.begin(115200);
    wifi.connect(WIFI_POWER_5dBm);
    lcd.init();
    accelerometer.init();

    exerciseBand.setCertificates(AMAZON_ROOT_CA1, CERTIFICATE, PRIVATE_KEY);
    exerciseBand.connectMQTT();

    pulseOximeter.init();
}

void loop()
{
    if (!exerciseBand.isConnected())
    {
        exerciseBand.connectMQTT();
    }
    exerciseBand.loop();

    pulseOximeter.update();

    if (millis() - tsLastReport > REPORTING_PERIOD_MS)
    {
        unsigned int pulse = pulseOximeter.getHeartRate();
        unsigned int pulseOx = pulseOximeter.getSpO2();
        exerciseBand.reportStateIfChanged(pulse);
        Serial.print("Heart rate:");
        Serial.print(pulse);
        Serial.print("bpm / SpO2:");
        Serial.print(pulseOx);
        Serial.println("%");

        tsLastReport = millis();
        lcd.printMessage(pulse, 0, 0, exerciseBand.getMessage());

        Serial.print("Step count: ");
        Serial.println(accelerometer.getStepCount());
    }

    accelerometer.update_steps();
}