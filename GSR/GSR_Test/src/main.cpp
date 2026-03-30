#include <Arduino.h>

const int GSR_PIN = A0;
float filtered = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  int raw = analogRead(GSR_PIN);

  filtered = 0.9 * filtered + 0.1 * raw;

  Serial.print("Filtered GSR: ");
  Serial.println(filtered);

  delay(100);
}
