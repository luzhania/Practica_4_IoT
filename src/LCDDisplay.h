#include <LiquidCrystal_I2C.h>

#pragma once

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

    void printMessage(unsigned int pulse, unsigned int pulseOx, unsigned int steps, String activityType, unsigned int row, unsigned int col, String message)
    {
        lcd.clear();
        lcd.setCursor(col, row);
        lcd.print(message.c_str());
        lcd.setCursor(0, 1);
        String pulseMessage = String(pulse) + " lpm / SpO2:" + String(pulseOx) + "%";
        lcd.print(pulseMessage);
        lcd.setCursor(0, 2);
        String stepsMessage = "Pasos: " + String(steps) + "/" + activityType;
        lcd.print(stepsMessage);
    }
};