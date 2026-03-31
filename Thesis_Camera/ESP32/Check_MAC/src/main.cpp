#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // wait for serial port to connect. Needed for native USB
  }
  
  delay(1000);

  WiFi.mode(WIFI_STA);   // enable Wi-Fi station mode

  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {}