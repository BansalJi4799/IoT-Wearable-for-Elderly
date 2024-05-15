#include <TinyGPS.h>
#include <SPI.h>
#include <SD.h>
#include <ArduinoBLE.h>

TinyGPS gps;

long lat, lon;
float startLat, startLon;
float prevLatitude, prevLongitude;
unsigned long prevTime;
float prevSpeed;

const int chipSelect = 10;
File dataFile;

unsigned long previousMillis = 0;
const long interval = 10000; // Set the interval to 10 seconds

void setup()
{
    delay(5000);
    Serial.begin(9600);  // Initialize Serial for debugging
    Serial1.begin(9600); // Initialize Serial1 for GPS communication

    Serial.println("Initializing SD card...");
    if (!SD.begin(chipSelect))
    {
        Serial.println("Card failed, or not present");
        while (1)
            ;
    }
    Serial.println("card initialized.");

    // Create or open the text file on the SD card
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile)
    {
        dataFile.println("Latitude, Longitude, Distance, Speed, Acceleration, Heart_rate");

        dataFile.close();
    }
    else
    {
        Serial.println("Error opening text file");
    }
}

void loop()
{
    while (Serial1.available())
    {
        if (gps.encode(Serial1.read()))
        {
            gps.get_position(&lat, &lon);
            float latitude = lat / 1000000.0;
            float longitude = lon / 1000000.0;

            // If this is the first GPS reading, set it as the starting position
            if (startLat == 0 && startLon == 0)
            {
                startLat = latitude;
                startLon = longitude;
            }

            // Calculate distance using the haversine formula
            float distance = calculateDistance(latitude, longitude, startLat, startLon);

            // Calculate speed
            float speed = calculateSpeed(startLat, startLon, latitude, longitude, prevTime, millis());

            // Calculate acceleration
            float acceleration = calculateAcceleration(prevSpeed, speed, prevTime, millis());

            // Read data from BLE
            String bleData = " ";
            readBLEData(bleData);

            Serial.println(bleData);
            Serial.println(distance, 6);
            Serial.println(speed, 6);
            // Log data to SD card
            logDataToSD(latitude, longitude, distance, speed, acceleration, bleData);

            // Update previous GPS data and speed for the next iteration
            prevLatitude = latitude;
            prevLongitude = longitude;
            prevTime = millis();
            prevSpeed = speed;

            // Wait for 10 seconds before the next reading
            delay(100);
        }
    }
}

void sendSDCardData()
{
    String receivedData = "";
    // Open the text file on the SD card
    dataFile = SD.open("datalog.txt");
    if (dataFile)
    {
        while (dataFile.available() && BLE.available())
        {
            char dataChar = dataFile.read();
            receivedData += dataChar;
            BLE.print(dataChar);
        }
        dataFile.close();
    }
}

float calculateDistance(float lat2, float lon2, float lat1, float lon1)
{
    // Haversine formula for calculating distance between two points on the Earth's surface
    // Radius of the Earth in kilometers
    float R = 6371.0;

    // Convert latitude and longitude from degrees to radians
    lat1 = radians(lat1);
    lon1 = radians(lon1);
    lat2 = radians(lat2);
    lon2 = radians(lon2);

    // Differences in coordinates
    float dlat = lat2 - lat1;
    float dlon = lon2 - lon1;

    // Haversine formula
    float a = sin(dlat / 2.0) * sin(dlat / 2.0) + cos(lat1) * cos(lat2) * sin(dlon / 2.0) * sin(dlon / 2.0);
    float c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    float distance = R * c;

    return distance;
}

float calculateSpeed(float lat1, float lon1, float lat2, float lon2, unsigned long time1, unsigned long time2)
{
    // Calculate distance using Haversine formula
    float distance = calculateDistance(lat1, lon1, lat2, lon2);

    // Calculate speed
    float speed = distance / ((time2 - time1) / 1000.0); // Convert time to seconds
    return speed;
}

float calculateAcceleration(float speed1, float speed2, unsigned long time1, unsigned long time2)
{
    // Calculate acceleration
    float acceleration = (speed2 - speed1) / ((time2 - time1) / 1000.0); // Convert time to seconds
    return acceleration;
}

void readBLEData(String &receivedData)
{
    receivedData = ""; // Initialize with an empty string

    // Read data from BLE
    while (BLE.available())
    {
        char rChar = BLE.read();
        receivedData += rChar;
    }
}

void logDataToSD(float latitude, float longitude, float distance, float speed, float acceleration, String bleData)
{
    // Open the text file
    dataFile = SD.open("datalog.txt", FILE_WRITE);

    // If the file is available, write data to it
    if (dataFile)
    {
        dataFile.print(latitude, 6);
        dataFile.print(", ");
        dataFile.print(longitude, 6);
        dataFile.print(", ");
        dataFile.print(distance, 6);
        dataFile.print(", ");
        dataFile.print(speed, 6);
        dataFile.print(", ");
        dataFile.print(acceleration, 6);
        dataFile.print(", ");
        dataFile.println(bleData);

        dataFile.close();
    }
    else
    {
        Serial.println("Error opening text file");
    }
}
