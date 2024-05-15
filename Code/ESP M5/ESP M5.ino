#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS 1000

PulseOximeter pox;
uint32_t tsLastReport = 0;

void onBeatDetected()
{
    Serial.println("Beat!!!");
}

void setup()
{
    Serial.begin(115200);

    if (!pox.begin())
    {
        Serial.println("FAILED");
        while (1)
            ;
    }
    else
    {
        Serial.println("SUCCESS");
    }
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

    pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop()
{
    pox.update();
    if (millis() - tsLastReport > REPORTING_PERIOD_MS)
    {
        Serial.print("BPM: ");
        Serial.print(pox.getHeartRate());
        Serial.print(" SpO2: ");
        Serial.print(pox.getSpO2());
        Serial.println("%");

        tsLastReport = millis();
    }
}
