#include <Arduino.h>
#include <ArduinoBLE.h> // BLE 라이브러리 추가

// --- 시뮬레이션 설정 ---
const int SIMULATION_WINDOW_SECONDS = 10;   // 데이터 수집 및 분석 시간 (초)
const int PPG_SAMPLE_RATE_HZ = 100;         // PPG 센서 샘플링 주파수 (Hz)
const int GSR_SAMPLE_RATE_HZ = 10;          // GSR 센서 샘플링 주파수 (Hz)

// --- 데이터 타입 정의 ---
using ppg_raw_t = uint32_t; // 18-bit PPG 데이터는 32-bit(4바이트) 변수에 저장
using gsr_raw_t = uint16_t; // 12-bit GSR 데이터는 16-bit(2바이트) 변수에 저장

// --- BLE 서비스 및 특성(Characteristic) UUID 정의 ---
BLEService        bioDataService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic statusChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLENotify, 32);
BLECharacteristic rawDataChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLENotify, 20);
BLECharacteristic featureDataChar("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 24);
BLEUnsignedCharCharacteristic controlPointChar("19B10004-E8F2-537E-4F6C-D104768A1214", BLEWrite);

// --- 데이터 구조체 정의 ---
// #pragma pack(1): 구조체 멤버 사이에 불필요한 빈 공간(padding)을 넣지 않아 데이터 크기를 정확하게 제어

#pragma pack(1)
// Raw 데이터 전송용 패킷 구조체
struct RawDataPacket {
  uint8_t type;   // 'P' for PPG, 'G' for GSR
  uint32_t value; // 18-bit, 12-bit 값을 모두 담을 수 있도록 uint32_t 사용
}; // 총 5 바이트

// Feature 데이터 구조체
struct FeatureData {
  float ppg_mean_hr;    // 4 bytes
  float ppg_sdnn;       // 4 bytes
  float ppg_rmssd;      // 4 bytes
  float gsr_scl_mean;   // 4 bytes
  float gsr_scr_amp;    // 4 bytes
  float gsr_nsscr_freq; // 4 bytes
}; // 총 24 바이트
#pragma pack()

void runDataRateSimulation();

void setup() {
  // 시리얼 통신 시작 (디버깅용)
  Serial.begin(115200);
  while (!Serial); // 개발 중에는 이 라인을 주석 처리하여 시리얼 연결 없이도 부팅되도록 함
  Serial.println("Bio-Grip Data Rate Analyzer - BLE Initializing...");

  // BLE 초기화
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1); // BLE 시작 실패 시 무한 루프
  }

  // BLE 장치 이름 설정
  BLE.setLocalName("BioGrip Analyzer");
  // BLE 서비스 및 특성 설정
  BLE.setAdvertisedService(bioDataService);
  bioDataService.addCharacteristic(statusChar);
  bioDataService.addCharacteristic(rawDataChar);
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

  // 제어 지점(Control Point)에 스마트폰으로부터 명령이 수신되었는지 확인
  if (controlPointChar.written()) {
    uint8_t command = controlPointChar.value();
    
    switch(command) {
      case 1:
        Serial.println("COMMAND[1]: Received 'Run Data Rate Simulation'.");
        runDataRateSimulation();
        break;
      default:
        Serial.print("COMMAND[?]: Received unknown command: ");
        Serial.println(command);
        statusChar.writeValue("Error: Unknown command");
        break;
    }
  }
}

/**
 * @brief 데이터 전송량 절감율 측정을 위한 전체 시뮬레이션을 실행합니다.
 * 1. Raw 데이터와 Feature 데이터의 이론적인 크기를 계산합니다.
 * 2. Raw 데이터를 BLE로 스트리밍하는 것을 시뮬레이션합니다.
 * 3. Feature 데이터를 BLE로 전송하는 것을 시뮬레이션합니다.
 * 4. 최종 절감율을 계산하여 시리얼 모니터에 출력합니다.
 */
void runDataRateSimulation() {
  statusChar.writeValue("Starting simulation...");
  delay(100);

  const int WINDOW_SIZE_SEC = 1;  // 1초 윈도우
  const int NUM_WINDOWS = SIMULATION_WINDOW_SECONDS / WINDOW_SIZE_SEC; // 10회

  // 1초당 샘플 수
  const int PPG_SAMPLES_PER_WINDOW = PPG_SAMPLE_RATE_HZ * WINDOW_SIZE_SEC; // 100개
  const int GSR_SAMPLES_PER_WINDOW = GSR_SAMPLE_RATE_HZ * WINDOW_SIZE_SEC; // 10개

  long total_raw_bytes = 0;
  long total_feature_bytes = 0;

  Serial.println("\n=== Simulation Start ===");
  Serial.print("Windows: "); Serial.print(NUM_WINDOWS);
  Serial.print(" x "); Serial.print(WINDOW_SIZE_SEC); Serial.println("sec");

  // ── 핵심: 1초 윈도우 반복 ──
  for (int w = 0; w < NUM_WINDOWS; w++) {
    Serial.print("\n--- [Window "); Serial.print(w + 1);
    Serial.print("/"); Serial.print(NUM_WINDOWS); Serial.println("] ---");

    char statusMsg[32];
    snprintf(statusMsg, sizeof(statusMsg), "Window %d/%d collecting...", w + 1, NUM_WINDOWS);
    statusChar.writeValue(statusMsg);

    // PPG 샘플 배열 (Feature 계산용)
    ppg_raw_t ppg_buf[PPG_SAMPLES_PER_WINDOW];
    gsr_raw_t gsr_buf[GSR_SAMPLES_PER_WINDOW];

    // --- 1. PPG 수집 + Raw 전송 ---
    for (int i = 0; i < PPG_SAMPLES_PER_WINDOW; i++) {
      ppg_buf[i] = random(0, 262143);

      RawDataPacket p;
      p.type = 'P';
      p.value = ppg_buf[i];
      rawDataChar.writeValue((uint8_t*)&p, sizeof(p));
      delay(2);
    }
    total_raw_bytes += PPG_SAMPLES_PER_WINDOW * sizeof(ppg_raw_t);

    // --- 2. GSR 수집 + Raw 전송 ---
    for (int i = 0; i < GSR_SAMPLES_PER_WINDOW; i++) {
      gsr_buf[i] = random(0, 4095);

      RawDataPacket p;
      p.type = 'G';
      p.value = gsr_buf[i];
      rawDataChar.writeValue((uint8_t*)&p, sizeof(p));
      delay(2);
    }
    total_raw_bytes += GSR_SAMPLES_PER_WINDOW * sizeof(gsr_raw_t);

    // --- 3. Feature 계산 ---
    // PPG: Mean, SDNN, RMSSD 계산
    float ppg_sum = 0;
    for (int i = 0; i < PPG_SAMPLES_PER_WINDOW; i++) ppg_sum += ppg_buf[i];
    float ppg_mean = ppg_sum / PPG_SAMPLES_PER_WINDOW;

    float sdnn_sum = 0;
    for (int i = 0; i < PPG_SAMPLES_PER_WINDOW; i++) {
      float diff = ppg_buf[i] - ppg_mean;
      sdnn_sum += diff * diff;
    }
    float ppg_sdnn = sqrt(sdnn_sum / PPG_SAMPLES_PER_WINDOW);

    float rmssd_sum = 0;
    for (int i = 1; i < PPG_SAMPLES_PER_WINDOW; i++) {
      float diff = (float)ppg_buf[i] - (float)ppg_buf[i - 1];
      rmssd_sum += diff * diff;
    }
    float ppg_rmssd = sqrt(rmssd_sum / (PPG_SAMPLES_PER_WINDOW - 1));

    // GSR: SCL Mean, SCR Amp, NS-SCR Freq 계산
    float gsr_sum = 0;
    gsr_raw_t gsr_min = gsr_buf[0], gsr_max = gsr_buf[0];
    int scr_count = 0;
    for (int i = 0; i < GSR_SAMPLES_PER_WINDOW; i++) {
      gsr_sum += gsr_buf[i];
      if (gsr_buf[i] > gsr_max) gsr_max = gsr_buf[i];
      if (gsr_buf[i] < gsr_min) gsr_min = gsr_buf[i];
      // 단순 SCR 카운트: 전 샘플보다 10 이상 오르면 카운트
      if (i > 0 && (gsr_buf[i] - gsr_buf[i-1]) > 10) scr_count++;
    }
    float gsr_scl_mean = gsr_sum / GSR_SAMPLES_PER_WINDOW;
    float gsr_scr_amp  = gsr_max - gsr_min;
    float gsr_nsscr    = (float)scr_count / WINDOW_SIZE_SEC;

    // HR 추정 (시뮬레이션이므로 mean을 간단히 스케일링)
    float mean_hr = 60.0f + (ppg_mean / 262143.0f) * 60.0f; // 60~120 bpm 범위

    // --- 4. Feature 패킷 전송 ---
    FeatureData feat;
    feat.ppg_mean_hr    = mean_hr;
    feat.ppg_sdnn       = ppg_sdnn;
    feat.ppg_rmssd      = ppg_rmssd;
    feat.gsr_scl_mean   = gsr_scl_mean;
    feat.gsr_scr_amp    = gsr_scr_amp;
    feat.gsr_nsscr_freq = gsr_nsscr;

    featureDataChar.writeValue((uint8_t*)&feat, sizeof(feat));
    total_feature_bytes += sizeof(FeatureData);

    Serial.print(" HR:"); Serial.print(feat.ppg_mean_hr, 1);
    Serial.print(" SDNN:"); Serial.print(feat.ppg_sdnn, 1);
    Serial.print(" RMSSD:"); Serial.print(feat.ppg_rmssd, 1);
    Serial.print(" SCL:"); Serial.print(feat.gsr_scl_mean, 1);
    Serial.print(" SCR_A:"); Serial.print(feat.gsr_scr_amp, 1);
    Serial.print(" NSSCR:"); Serial.println(feat.gsr_nsscr_freq, 2);
    Serial.println(" => Feature packet sent.");

    delay(50); // 다음 윈도우 전 BLE 스택 안정화
  }

  // --- 최종 결과 ---
  float reduction = (1.0f - ((float)total_feature_bytes / total_raw_bytes)) * 100.0f;

  Serial.println("\n\n=== [FINAL RESULT] ===");
  Serial.print("Total Raw:     "); Serial.print(total_raw_bytes); Serial.println(" bytes");
  Serial.print("Total Feature: "); Serial.print(total_feature_bytes); Serial.println(" bytes");
  Serial.print("Reduction:     "); Serial.print(reduction, 2); Serial.println(" %");
  Serial.println("======================");

  char finalStatus[32];
  snprintf(finalStatus, sizeof(finalStatus), "Done! Red: %.2f%%", reduction);
  statusChar.writeValue(finalStatus);
}
