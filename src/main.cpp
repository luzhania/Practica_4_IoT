#include "WiFiConnection.h"
#include "config.h"
#include "Utilities.h"
#include "PulseOximeterSensor.h"
#include "AccelerometerSensor.h"
#include "LCDDisplay.h"
#include "MQTTClient.h"
#include "ExerciseBand.h"

#define REPORTING_PERIOD_MS 600

WiFiConnection wifi(WIFI_SSID, WIFI_PASS);
ExerciseBand exerciseBand(MQTT_BROKER, MQTT_PORT);

LCDDisplay lcd;
AccelerometerSensor accelerometer;

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

    Utilities::nonBlockingDelay(REPORTING_PERIOD_MS, []()
                                {
        unsigned int pulse = pulseOximeter.getHeartRate();
        unsigned int pulseOx = pulseOximeter.getSpO2();
        unsigned int steps = accelerometer.getStepCount();
        unsigned int temperature = accelerometer.getTemperature();
        unsigned int accelerometerMagnitude = accelerometer.getAcceloremeterMagnitude();
        exerciseBand.reportData(pulse, pulseOx, steps, temperature, accelerometerMagnitude);

        
        Serial.print("Heart rate:");
        Serial.print(pulse);
        Serial.print("bpm / SpO2:");
        Serial.print(pulseOx);
        Serial.println("%");
        Serial.print("Step count: ");
        Serial.println(accelerometer.getStepCount());

        lcd.printMessage(pulse, pulseOx, steps, exerciseBand.getActivityType(), 0, 0, exerciseBand.getMessage());
        });

    accelerometer.updateSteps();
}