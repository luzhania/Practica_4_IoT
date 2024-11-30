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