#include <WiFi.h>

#pragma once

class WiFiConnection
{
private:
    const char *SSID;
    const char *PASSWORD;
    bool isConnected = false;

public:
    WiFiConnection(const char *SSID, const char *PASSWORD)
        : SSID(SSID), PASSWORD(PASSWORD) {}

    // void connect(unsigned long timeout = 10000, wifi_power_t txPower = WIFI_POWER_8_5dBm) {
    //     WiFi.begin(SSID, PASSWORD);

    //     WiFi.setTxPower(txPower);

    //     unsigned long startAttemptTime = millis();

    //     while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
    //         delay(500);
    //         Serial.print(".");
    //     }

    //     if (WiFi.status() == WL_CONNECTED) {
    //         Serial.println("\nConnected to WiFi");
    //         isConnected = true;
    //         Serial.print("IP Address: ");
    //         Serial.println(WiFi.localIP());
    //     } else {
    //         Serial.println("\nFailed to connect to WiFi");
    //         isConnected = false;
    //     }
    // }
    void connect(wifi_power_t txPower = WIFI_POWER_8_5dBm)
    {
        WiFi.begin(SSID, PASSWORD);
        WiFi.setTxPower(txPower);
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(10);
            Serial.println("Connecting to WiFi...");
        }
        Serial.println("Connected to WiFi");
    }
};