#include <Arduino.h>
#include <ArduinoBLE.h> // BLE 라이브러리 추가

// --- BLE 프레임 설정 ---
// 서비스와 각 특성은 고유한 UUID를 가져야 합니다.
BLEService bioDataService("19B10000-E8F2-537E-4F6C-D104768A1214"); // 생체 데이터 서비스

// 1. 특징 데이터 특성 (Feature Data Characteristic)
//    - 장치 -> 스마트폰으로 계산된 특징 데이터를 전송합니다.
//    - (Read: 현재 값 읽기, Notify: 값이 변경될 때마다 자동 알림)
BLECharacteristic featureDataChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 20); // 최대 20바이트

// 2. 제어 지점 특성 (Control Point Characteristic)
//    - 스마트폰 -> 장치로 명령을 전송합니다. (예: 측정 시작/중지)
//    - (Write: 스마트폰에서 값을 쓸 수 있음)
BLEUnsignedCharCharacteristic controlPointChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLEWrite);

// --- 데이터 구조 정의 ---
// 전송될 특징 데이터를 바이너리 구조체로 정의합니다.
// #pragma pack(1)은 컴파일러가 구조체 멤버 사이에 불필요한 패딩(padding)을 넣지 않도록 하여
// 데이터 크기를 정확하게 제어합니다.
#pragma pack(1)
struct FeatureData {
  float heart_rate;      // 4 바이트
  float skin_conductance; // 4 바이트
  uint8_t scr_count;     // 1 바이트
}; // 총 9 바이트
#pragma pack()

// 전역 변수로 특징 데이터 구조체 선언
FeatureData currentFeatures;

void setup() {
  // 시리얼 통신 시작 (디버깅용)
  Serial.begin(115200);
  while (!Serial) {
    ; // 시리얼 포트가 연결될 때까지 대기
  }
  Serial.println("Bio-Grip BLE Feature-Data Framework");

  // BLE 초기화
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1); // BLE 시작 실패 시 무한 루프
  }

  // BLE 장치 이름 설정
  BLE.setLocalName("BioGrip_Feature");
  // BLE 서비스 및 특성 설정
  BLE.setAdvertisedService(bioDataService);
  bioDataService.addCharacteristic(featureDataChar);
  bioDataService.addCharacteristic(controlPointChar);
  BLE.addService(bioDataService);

  // Advertise 시작
  BLE.advertise();

  Serial.println("BLE 장치 활성화 완료. 스마트폰 앱(nRF Connect)으로 연결을 기다립니다...");
}

void loop() {
  // BLE 이벤트를 처리합니다. (필수)
  BLE.poll();

  // 제어 지점(Control Point)에 값이 쓰여졌는지 확인합니다.
  if (controlPointChar.written()) {
    // 스마트폰에서 보낸 명령 값을 읽습니다.
    uint8_t command = controlPointChar.value();
    Serial.print("명령 수신: ");
    Serial.println(command);

    // 명령 1: 측정 시작 및 데이터 전송
    if (command == 1) {
      Serial.println("측정 시작 명령 수신. 특징 값 계산 및 전송을 시작합니다.");

      // --- 여기에 실제 센서 측정 및 특징 추출 코드를 넣습니다 ---
      // 아래는 시뮬레이션입니다.
      // 10초간의 Raw 데이터를 분석하여 특징 값을 얻었다고 가정합니다.
      currentFeatures.heart_rate = random(60, 90) + (random(0, 100) / 100.0); // 예: 75.5 bpm
      currentFeatures.skin_conductance = random(2, 10) + (random(0, 100) / 100.0); // 예: 5.2 uS
      currentFeatures.scr_count = random(0, 5); // 예: 3회
      // ---------------------------------------------------------

      Serial.println("특징 값 계산 완료:");
      Serial.print(" - HR: "); Serial.println(currentFeatures.heart_rate);
      Serial.print(" - SCL: "); Serial.println(currentFeatures.skin_conductance);
      Serial.print(" - SCR Count: "); Serial.println(currentFeatures.scr_count);

      // 계산된 특징 데이터를 BLE 특성에 씁니다.
      // 구조체 변수의 주소와 크기를 넘겨주어 바이너리 형태로 전송합니다.
      featureDataChar.writeValue((uint8_t*)&currentFeatures, sizeof(currentFeatures));

      Serial.println("BLE를 통해 특징 데이터를 전송했습니다.");
    }
  }
}
