#include <TinyGPS++.h>
#include <Wire.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include "MAX30100_PulseOximeter.h"
#include <WiFi.h>
#include <WiFiNINA.h>

#define REPORTING_PERIOD_MS 1000
#define BUTTON_PIN 2

// Email settings (replace with your details)
const char* emailSender = "himanshu02092002@gmail.com";     // Replace with your email address
const char* emailRecipient = "yuvrajbansal3396@gmail.com";  // Replace with recipient email address
const char* emailPassword = "smph dgii qdez sipq";          // Replace with your email app password (not regular password)

static const uint32_t GPSBaud = 9600;

// WiFi settings
const char* ssid = "Google pixel 7 Pro";
const char* password = "PROMAN2$$2";

// MQTT settings
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_username = "";
const char* mqtt_password = "";
const char* mqtt_topic = "iot/data";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

TinyGPSPlus gps;

PulseOximeter pox;
uint32_t tsLastReport = 0;

float prevLat = 0, prevLon = 0;
unsigned long prevTime = 0;
float prevSpeed = 0;
float prevHeartRate = 0;

// Global variables
float currLat = 0, currLon = 0;
float distance = 0, speed = 0;
float heartRate = 0;

void setup() {
  Serial.begin(9600);      // Initialize hardware serial
  Serial1.begin(GPSBaud);  // Initialize serial communication for GPS module
  if (!pox.begin()) {
    Serial.println("FAILED");
    while (1)
      ;
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

  // Connect to WiFi
  connectWiFi();

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Set MQTT server and callback function
  mqttClient.setServer(mqtt_server, mqtt_port);
}

void loop() {
  while (Serial1.available() > 0)
    if (gps.encode(Serial1.read())) {
      displayInfo();
    }

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS detected: check wiring."));
    while (true)
      ;
  }

  if (digitalRead(BUTTON_PIN) == LOW) {
    sendSOSEmail();  // Call function to send SOS email
    delay(5000);     // Debounce button press (optional)
  }

  // Serial.println(digitalRead(BUTTON_PIN));

  pox.update();
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {

    heartRate = pox.getHeartRate();
    // Send data over MQTT
    publishDataToMQTT(heartRate);

    // Print data to Serial monitor
    printDataToSerial();

    tsLastReport = millis();
  }
}

void displayInfo() {
  if (gps.location.isValid()) {
    pox.update();

    currLat = gps.location.lat();
    currLon = gps.location.lng();

    // Calculate distance and speed only if there's a significant change in latitude or longitude
    if (abs(currLat - prevLat) > 0.000001 || abs(currLon - prevLon) > 0.000001) {
      // Calculate distance using the haversine formula
      distance = calculateDistance(currLat, currLon, prevLat, prevLon);

      // Calculate time difference in seconds
      float timeDiffSeconds = (millis() - prevTime) / 1000.0;  // Convert milliseconds to seconds

      // Calculate speed in meters per second
      speed = distance / timeDiffSeconds;

      // Update previous GPS data and time for the next iteration
      prevLat = currLat;
      prevLon = currLon;
      prevTime = millis();
      prevSpeed = speed;
    }
  }
}

void connectWiFi() {
  Serial.println("Connecting to WiFi");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected");
}

void publishDataToMQTT(float heartRate) {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }

  // Construct message payload
  String payload = String(currLat, 6) + "," + String(currLon, 6) + "," + String(distance, 6) + "," + String(speed, 6) + "," + String(heartRate);

  // Publish payload to MQTT topic
  mqttClient.publish(mqtt_topic, payload.c_str());
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (mqttClient.connect("ArduinoClient")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

float calculateDistance(float lat2, float lon2, float lat1, float lon1) {
  // Haversine formula for calculating distance between two points on the Earth's surface
  // Radius of the Earth in meters
  float R = 6371000.0;

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

void printDataToSerial() {
  Serial.print("Latitude: ");
  Serial.print(currLat, 6);
  Serial.print(", Longitude: ");
  Serial.print(currLon, 6);
  Serial.print(", Distance (m): ");
  Serial.print(distance, 6);
  Serial.print(", Speed (m/s): ");
  Serial.print(speed, 6);
  Serial.print(", Heart Rate: ");
  Serial.println(heartRate);
}
// Function to send SOS email
void sendSOSEmail() {
  // Libraries for sending email are not included in the standard Arduino libraries.
  // You can explore third-party libraries like "SMTPClient" or "MailSender"
  // but be aware that their usage might require additional configuration
  // (like enabling less secure apps in your Gmail settings, which is not recommended).

  // This section provides a placeholder to show the structure of the email content.
  // You'll need to implement the actual email sending functionality using a chosen library.
  String emailBody = "SOS! This is an automated email from my Arduino Nano 33 IoT.\n";
  emailBody += "Current location (if GPS available):\n";
  emailBody += "  Latitude: ";
  emailBody += String(currLat, 6);
  emailBody += "\n  Longitude: ";
  emailBody += String(currLon, 6);

  // Implement sending the email using your chosen library here.
  // This is a placeholder, replace it with the actual sending logic.
  Serial.println("Sending SOS email...");
  Serial.println(emailBody);  // Print email content for debugging

  // Delay to avoid overwhelming the email service (optional)
  delay(2000);
}
