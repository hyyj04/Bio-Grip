#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

float beatsPerMinute = 0;
int beatAvg = 0;

unsigned long startTime;
unsigned long lastPrintTime = 0;

void setup()
{
  Serial.begin(115200);
  delay(1500);

  Serial.println("Initializing...");

  Wire.begin();

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("MAX30102 not found");
    while (1);
  }

  Serial.println("Sensor OK");
  Serial.println("Start measurement (60 seconds)");
  Serial.println("Place your finger on sensor");

  particleSensor.setup();

  // 안정적인 LED 세기
  particleSensor.setPulseAmplitudeRed(0x0F);
  particleSensor.setPulseAmplitudeIR(0x0F);

  startTime = millis();
}

void loop()
{
  // ⏱ 120초 후 종료
  if (millis() - startTime > 120000)
  {
    Serial.println("Measurement finished");
    while (1);
  }

  long irValue = particleSensor.getIR();

  // 💓 심박 검출
  if (checkForBeat(irValue) == true)
  {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  // 🕒 0.1초마다 출력 (100ms)
  if (millis() - lastPrintTime > 100)
  {
    lastPrintTime = millis();

    Serial.print("Time: ");
    Serial.print((millis() - startTime) / 1000.0, 1);
    Serial.print("s | IR=");
    Serial.print(irValue);
    Serial.print(" | BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(" | Avg BPM=");
    Serial.print(beatAvg);

    if (irValue < 50000)
      Serial.print(" | Weak signal");

    Serial.println();
  }
}
