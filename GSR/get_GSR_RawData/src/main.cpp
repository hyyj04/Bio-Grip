#include <Arduino.h>

const int GSR_PIN = A0;
const float ADC_REF = 3.3f;
const int ADC_MAX = 1023;

const unsigned long SAMPLE_INTERVAL_MS = 100;
const unsigned long DURATION_MS = 60000;  // 60초

unsigned long lastSampleTime = 0;
unsigned long startTime = 0;

bool isRunning = true;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  analogReadResolution(10);

  startTime = millis();

  Serial.println("time_s,gsr_raw,gsr_voltage");
}

void loop() {
  if (!isRunning) return;

  unsigned long now = millis();

  // 60초 지나면 종료
  if (now - startTime >= DURATION_MS) {
    isRunning = false;
    Serial.println("END");
    return;
  }

  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime += SAMPLE_INTERVAL_MS;

    int raw = analogRead(GSR_PIN);
    float voltage = (raw * ADC_REF) / ADC_MAX;
    float time_s = (now - startTime) / 1000.0;

    Serial.print(time_s, 3);
    Serial.print(",");
    Serial.print(raw);
    Serial.print(",");
    Serial.println(voltage, 4);
  }
}
