#include <Wire.h> //I2C 통신 라이브러리
#include "MAX30105.h" //MAX30102 센서 제어 라이브러리

MAX30105 sensor; // 센서 객체 생성

// 측정 시간: 60초 (마이크로초 단위)
const unsigned long MEASURE_DURATION_US = 60000000UL;

// 400 Hz = 1 / 400 = 0.0025 s = 2500 us
const unsigned long SAMPLE_INTERVAL_US = 2500UL;

// 시간 변수
unsigned long startTime_us = 0;       // 측정 시작 시간
unsigned long lastSampleTime_us = 0;  // 마지막 샘플링 시점

void setup()
{
  Serial.begin(115200);   //시리얼 통신 시작
  delay(1500);            // 센서 안정화 대기

  Wire.begin(); // I2C 통신 시작

  // 센서 초기화 실패 시 루프
  Serial.println("Initializing MAX30102...");

  if (!sensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("MAX30102 not found");
    while (1);
  }

  // MAX30102 센서 설정
  byte ledBrightness = 0x5F;   // LED 전류 세기 (약 19mA) 전류 세기는 Red & IR LED에 동시에 적용됨
  byte sampleAverage = 4;      // 4개 샘플 평균
  byte ledMode = 2;            // Heart Rate mode로 설정 (Red only)
  int sampleRate = 400;        // 400 Hz로 샘플링



  int pulseWidth = 411;        // 411 us -> 18 bit 해상도를 사용하기 위한 설정
  int adcRange = 16384;        // 최대 ADC range

  // MAX30102 센서 설정 적용
  sensor.setup(
    ledBrightness,
    sampleAverage,
    ledMode,
    sampleRate,
    pulseWidth,
    adcRange
  );

  // 시리얼 모니터 출력
  Serial.println("Start PPG RAW Measurement (60s)");
  Serial.println("time_ms,ir_value");

  // 시작 시간 기록
  startTime_us = micros(); 
  lastSampleTime_us = startTime_us; 
}

void loop()
{
  unsigned long currentTime_us = micros();    // 현재 시간

  // 60초 측정 후 종료
  if (currentTime_us - startTime_us >= MEASURE_DURATION_US)
  {
    Serial.println("END");
    while (1);
  }

  // 400Hz 주기로 샘플링 진행
  if (currentTime_us - lastSampleTime_us >= SAMPLE_INTERVAL_US)
  {
    lastSampleTime_us += SAMPLE_INTERVAL_US;

    long irValue = sensor.getIR();    // IR 데이터 읽기

    // CSV 형식으로 출력 (시간(ms), IR 값)
    Serial.print((currentTime_us - startTime_us) / 1000);    // ms
    Serial.print(",");  
    Serial.println(irValue);
  }
}
