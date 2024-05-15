//#include <SoftwareSerial.h>
//#include <TinyGPS.h>
//
//long lat, lon;
//SoftwareSerial gpsSerial(4, 3); // Connect GPS sensor
//TinyGPS gps;
//
//void setup() {
//  Serial.begin(115200); // Connect Serial
//  gpsSerial.begin(9600); // Connect GPS sensor
//}
//
//void loop() {
//  while (gpsSerial.available()) {
//    if (gps.encode(gpsSerial.read())) {
//      gps.get_position(&lat, &lon);
//
//      // Print data to Serial in CSV format
//      Serial.print(lat / 1000000.0, 6);
//      Serial.print(",");
//      Serial.print(lon / 1000000.0, 6);
//      Serial.println();
//
//      delay(10000); // Wait for 10 seconds
//    }
//  }
//}




#include <TinyGPS.h>

long lat, lon;
TinyGPS gps;

void setup() {
  Serial.begin(115200); // Connect Serial
}

void loop() {
  while (Serial.available()) {
    if (gps.encode(Serial.read())) {
      gps.get_position(&lat, &lon);

      // Print data to Serial in CSV format
      Serial.print(lat / 1000000.0, 6);
      Serial.print(",");
      Serial.print(lon / 1000000.0, 6);
      Serial.println();

      delay(10000); // Wait for 10 seconds
    }
  }
}
