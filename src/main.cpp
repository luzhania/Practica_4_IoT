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

class MQTTClient{
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
                if (inputDoc["state"]["pulse_requested"] == 1)
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
        serializeJson(outputDoc, outputBuffer);
        client.publish(UPDATE_TOPIC, outputBuffer);
    }

    void publishPulseRequestAttended()
    {
        outputDoc.clear();
        outputDoc["state"]["desired"]["pulse_requested"] = 0;
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
Adafruit_MPU6050 mpu;
// Parameters for step counting
float threshold = 9.5;   // Adjust based on testing
unsigned long lastStepTime = 0;
int stepCount = 0;
unsigned long debounceTime = 300; // Minimum time (ms) between steps
float offsetX = 0, offsetY = 0;
void calculateOffsets() {
  const int numReadings = 100;
  float sumX = 0, sumY = 0;

  Serial.println("Calculating offsets...");

  for (int i = 0; i < numReadings; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

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


// Create a PulseOximeter object
PulseOximeter pox;

// Time at which the last beat occurred
uint32_t tsLastReport = 0;

// Callback routine is executed when a pulse is detected
void onBeatDetected() {
    Serial.println("Beat!");
}

void setup() {
    Serial.begin(115200);
   wifi.connect(WIFI_POWER_5dBm);
   lcd.init();

     while (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    delay(10);
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
   mpu.setGyroRange(MPU6050_RANGE_500_DEG);
     mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
     calculateOffsets();
    
    exerciseBand.setCertificates(AMAZON_ROOT_CA1, CERTIFICATE, PRIVATE_KEY);
    exerciseBand.connectMQTT();

    Serial.print("Initializing pulse oximeter..");

    // Initialize sensor
    if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }
  pox.setIRLedCurrent(MAX30100_LED_CURR_46_8MA);

    // Register a callback routine
    pox.setOnBeatDetectedCallback(onBeatDetected);

    
   

}

void loop() {
    if (!exerciseBand.isConnected())
    {
        exerciseBand.connectMQTT();
    }
    exerciseBand.loop();
    // Read from the sensor

    pox.update();

    // Grab the updated heart rate and SpO2 levels
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        unsigned int pulse = pox.getHeartRate();
        exerciseBand.reportStateIfChanged(pulse);
        Serial.print("Heart rate:");
        Serial.print(pulse);
        Serial.print("bpm / SpO2:");
        Serial.print(pox.getSpO2());
        Serial.println("%");

        tsLastReport = millis();
        lcd.printMessage(pulse, 0, 0, exerciseBand.getMessage());
    }

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float magnitude = sqrt(
    (a.acceleration.x - offsetX) * (a.acceleration.x - offsetX)+
    (a.acceleration.y- offsetY) * (a.acceleration.y - offsetY) +
    a.acceleration.z * a.acceleration.z
  );
  Serial.print("magnituide: ");
  Serial.println(magnitude);
  if (magnitude > threshold) {
    unsigned long currentTime = millis();
    if (currentTime - lastStepTime > debounceTime) {
      stepCount++;
      lastStepTime = currentTime;
      Serial.print("Step detected! Total steps: ");
      Serial.println(stepCount);
    }
  }
}