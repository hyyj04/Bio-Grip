#include <Wire.h>
#include "MAX30105.h"

MAX30105 sensor;

// 측정 시간: 60초
const unsigned long MEASURE_DURATION_US = 60000000UL;

// 400 Hz = 1 / 400 = 0.0025 s = 2500 us
const unsigned long SAMPLE_INTERVAL_US = 2500UL;

// 시간 변수
unsigned long startTime_us = 0;
unsigned long lastSampleTime_us = 0;

void setup()
{
  Serial.begin(115200);
  delay(1500);

  Wire.begin();

  Serial.println("Initializing MAX30102...");

  if (!sensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("MAX30102 not found");
    while (1);
  }

  // 추천 설정
  byte ledBrightness = 0x5F;   // LED 세기
  byte sampleAverage = 4;      // 평균 횟수
  byte ledMode = 2;            // Red + IR
  int sampleRate = 400;        // 400 Hz
  int pulseWidth = 411;        // 411 us -> 18 bit
  int adcRange = 16384;        // 최대 ADC range

  sensor.setup(
    ledBrightness,
    sampleAverage,
    ledMode,
    sampleRate,
    pulseWidth,
    adcRange
  );

  Serial.println("Start PPG RAW Measurement (60s)");
  Serial.println("time_ms,ir_value");

  startTime_us = micros();
  lastSampleTime_us = startTime_us;
}

void loop()
{
  unsigned long currentTime_us = micros();

  // 60초 측정 후 종료
  if (currentTime_us - startTime_us >= MEASURE_DURATION_US)
  {
    Serial.println("END");
    while (1);
  }

  // 400Hz 주기로 샘플링
  if (currentTime_us - lastSampleTime_us >= SAMPLE_INTERVAL_US)
  {
    lastSampleTime_us += SAMPLE_INTERVAL_US;

    long irValue = sensor.getIR();

    // CSV 형식 출력
    Serial.print((currentTime_us - startTime_us) / 1000); // ms
    Serial.print(",");
    Serial.println(irValue);
  }
}
