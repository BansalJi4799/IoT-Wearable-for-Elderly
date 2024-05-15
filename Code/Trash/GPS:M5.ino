#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <TinyGPS++.h>

#define REPORTING_PERIOD_MS 1000
#define GPSBaud 9600

PulseOximeter pox;
TinyGPSPlus gps;

uint32_t tsLastReport = 0;

void onBeatDetected()
{
    Serial.println("Beat!!!");
}

void setup()
{
    Serial.begin(115200);   // Initialize serial for debug messages
    Serial1.begin(GPSBaud); // Initialize serial communication for GPS module

    if (!pox.begin())
    {
        Serial.println("Pulse Oximeter FAILED");
        while (1)
            ;
    }
    else
    {
        Serial.println("Pulse Oximeter SUCCESS");
    }
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
    pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop()
{
    // Pulse Oximeter Update
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

    // GPS Update
    while (Serial1.available() > 0)
    {
        if (gps.encode(Serial1.read()))
            displayInfo();
    }

    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
        Serial.println(F("No GPS detected: check wiring."));
        while (true)
            ;
    }
}

void displayInfo()
{
    Serial.print(F("Location: "));
    if (gps.location.isValid())
    {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F("  Date "));
    if (gps.date.isValid())
    {
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.println();
}
