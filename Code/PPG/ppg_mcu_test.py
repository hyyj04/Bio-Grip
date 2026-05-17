/* test
 * PPG 센서 신호 처리 파이프라인 - 테스트 코드 v2
 * Arduino Nano 33 BLE + MAX30102
 *
 * [배선]
 * MAX30102 VCC → Arduino 3.3V
 * MAX30102 GND → Arduino GND
 * MAX30102 SDA → Arduino A4
 * MAX30102 SCL → Arduino A5
 *
 * [센서 설정]
 * LED 밝기   : 0x5F (약 19mA)
 * 샘플 평균  : 4개 (센서 내부 평균 → 유효 100Hz)
 * LED 모드   : 2 (Red + IR)
 * 샘플 속도  : 400Hz
 * 펄스 폭    : 411us (18bit 해상도)
 * ADC 범위   : 16384
 *
 * [신호 처리 파이프라인]
 * 1. 데이터 수집  : 400Hz → 센서 내부 4개 평균 → 유효 100Hz
 * 2. 제1 전처리  : IIR DC Blocker (α=0.95, f_c≈0.80Hz)
 * 3. 제2 전처리  : Moving Average (N=5, f_null=20Hz)
 * 4. Onset 검출  : 동적 임계값 (150샘플 윈도우, 비율 0.4)
 *                  + 불응기 400ms
 *                  + IBI 유효 범위 500~1100ms
 *                  + 변화량 필터 20%
 * 5. RMSSD 연산  : 슬라이딩 윈도우 60개
 * 6. 구간 관리   : Warm-up 15초 → Baseline 30초 → 실측 45초~
 * 7. 정규화      : stress_PPG = clamp(0, (RMSSD_baseline - RMSSD_current) / RMSSD_baseline, 1)
 * 8. BLE 전송    : 9 bytes (Stress_Index + RMSSD + SCR_flag), 1초 고정 주기
 *
 * [버그 수정 v2]
 * 버그1: getIR() 직접 호출 → safeCheck(250) 내부 블로킹 → 250ms 후 0 반환
 *        → 0 < FINGER_THRESHOLD 오검출로 "손가락 떨어짐" 반복 발생
 *        수정: check() + available() + nextSample() 비블로킹 패턴으로 교체
 *
 * 버그2: resetFilters()에서 sigWin[], sigWinIdx, dynThresh, wasBelow 누락
 *        → 손가락 재접촉 시 이전 임계값으로 시작 → Onset 오검출
 *        수정: 해당 변수 초기화 추가
 */

#include <Wire.h>
#include "MAX30105.h"
#include <ArduinoBLE.h>

MAX30105 sensor;

// ═════════════════════════════════════════════════════
// BLE (9 bytes 패킷: Stress_Index + RMSSD + SCR_flag)
// ═════════════════════════════════════════════════════
BLEService ppgService("180D");
BLECharacteristic stressPacketChar("2A37", BLERead | BLENotify, 9);

// ═════════════════════════════════════════════════════
// 손가락 감지
// ═════════════════════════════════════════════════════
#define FINGER_THRESHOLD  50000
bool fingerDetected = false;

// ═════════════════════════════════════════════════════
// [단계 2] IIR DC Blocker (α=0.95)
// ═════════════════════════════════════════════════════
#define DC_ALPHA  0.95f
float dcXPrev = 0.0f;
float dcYPrev = 0.0f;

float applyDCBlocker(float x) {
  float y = x - dcXPrev + DC_ALPHA * dcYPrev;
  dcXPrev = x;
  dcYPrev = y;
  return y;
}

// ═════════════════════════════════════════════════════
// [단계 3] Moving Average (N=5)
// ═════════════════════════════════════════════════════
#define MA_N  5
float   maBuf[MA_N] = {0};
uint8_t maIdx = 0;
float   maSum = 0.0f;

float applyMovingAverage(float x) {
  maSum -= maBuf[maIdx];
  maBuf[maIdx] = x;
  maSum += x;
  maIdx = (maIdx + 1) % MA_N;
  return maSum / MA_N;
}

// ═════════════════════════════════════════════════════
// [단계 4] 동적 임계값 기반 Onset 검출
// ═════════════════════════════════════════════════════
#define WINDOW_SAMPLES   150
#define THRESHOLD_RATIO  0.4f
#define REFRACTORY_MS    400
#define IBI_MIN_MS       500.0f
#define IBI_MAX_MS       1100.0f
#define IBI_CHANGE_LIMIT 0.20f

float   sigWin[WINDOW_SAMPLES] = {0};
uint8_t sigWinIdx = 0;
float   dynThresh = 0.0f;
bool    wasBelow  = true;

unsigned long lastOnsetMs = 0;
bool          firstOnset  = true;

void updateThreshold(float sample) {
  sigWin[sigWinIdx] = sample;
  sigWinIdx = (sigWinIdx + 1) % WINDOW_SAMPLES;
  float pMax = sigWin[0], pMin = sigWin[0];
  for (int i = 1; i < WINDOW_SAMPLES; i++) {
    if (sigWin[i] > pMax) pMax = sigWin[i];
    if (sigWin[i] < pMin) pMin = sigWin[i];
  }
  dynThresh = pMin + (pMax - pMin) * THRESHOLD_RATIO;
}

// ═════════════════════════════════════════════════════
// [단계 5] IBI / RMSSD
// ═════════════════════════════════════════════════════
#define IBI_WINDOW  60
float   ibiBuf[IBI_WINDOW] = {0};
uint8_t ibiWriteIdx = 0;
uint8_t ibiCount    = 0;
float   latestRMSSD = 0.0f;

void addIBI(float ibi_ms) {
  ibiBuf[ibiWriteIdx] = ibi_ms;
  ibiWriteIdx = (ibiWriteIdx + 1) % IBI_WINDOW;
  if (ibiCount < IBI_WINDOW) ibiCount++;
}

float computeRMSSD() {
  if (ibiCount < 2) return 0.0f;
  int startIdx = (ibiWriteIdx - ibiCount + IBI_WINDOW) % IBI_WINDOW;
  float sumSqDiff = 0.0f;
  float prev = ibiBuf[startIdx];
  for (int i = 1; i < ibiCount; i++) {
    float curr = ibiBuf[(startIdx + i) % IBI_WINDOW];
    float diff = curr - prev;
    sumSqDiff += diff * diff;
    prev = curr;
  }
  return sqrt(sumSqDiff / (ibiCount - 1));
}

// ═════════════════════════════════════════════════════
// [단계 6] 구간 관리
// ═════════════════════════════════════════════════════
enum Phase { WARMUP, BASELINE, MEASURING };
Phase currentPhase = WARMUP;

unsigned long phaseStartMs = 0;
#define WARMUP_MS    15000UL
#define BASELINE_MS  30000UL

#define BASELINE_IBI_MAX      50
#define BASELINE_IBI_SD_LIMIT 200.0f

float   baselineIBIBuf[BASELINE_IBI_MAX] = {0};
uint8_t baselineIBICount = 0;
float   rmssdBaseline    = 0.0f;
bool    baselineValid    = false;

float computeIBIStdDev(float* buf, uint8_t n) {
  if (n < 2) return 0.0f;
  float sum = 0.0f;
  for (uint8_t i = 0; i < n; i++) sum += buf[i];
  float mean = sum / n;
  float sqSum = 0.0f;
  for (uint8_t i = 0; i < n; i++) {
    float d = buf[i] - mean;
    sqSum += d * d;
  }
  return sqrt(sqSum / (n - 1));
}

void resetBaseline() {
  baselineIBICount = 0;
  rmssdBaseline    = 0.0f;
  baselineValid    = false;
  ibiCount         = 0;
  ibiWriteIdx      = 0;
  Serial.println("[BASELINE] 품질 불량 → 재시작");
}

// ═════════════════════════════════════════════════════
// [단계 7] stress_PPG 정규화
// ═════════════════════════════════════════════════════
float stressPPG = 0.0f;

float clampF(float val, float lo, float hi) {
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
}

float computeStressPPG(float rmssdCurrent) {
  if (rmssdBaseline <= 0.0f) return 0.0f;
  return clampF((rmssdBaseline - rmssdCurrent) / rmssdBaseline, 0.0f, 1.0f);
}

// ═════════════════════════════════════════════════════
// [단계 8] BLE 전송
// ═════════════════════════════════════════════════════
#define BLE_INTERVAL_MS  1000UL
unsigned long lastBLEMs = 0;

void sendBLEPacket(float stressIndex, float rmssd, uint8_t scrFlag) {
  if (!BLE.connected()) return;
  uint8_t packet[9] = {0};
  memcpy(&packet[0], &stressIndex, 4);
  memcpy(&packet[4], &rmssd, 4);
  packet[8] = scrFlag;
  stressPacketChar.writeValue(packet, 9);
}

// ═════════════════════════════════════════════════════
// 전체 초기화 [버그 수정: sigWin/sigWinIdx/dynThresh/wasBelow 추가]
// ═════════════════════════════════════════════════════
void resetFilters() {
  dcXPrev = 0.0f;  dcYPrev = 0.0f;
  maSum = 0.0f;    memset(maBuf, 0, sizeof(maBuf));  maIdx = 0;
  // [수정] 신호 윈도우 완전 초기화
  memset(sigWin, 0, sizeof(sigWin));
  sigWinIdx = 0;  dynThresh = 0.0f;  wasBelow = true;
  firstOnset = true;  lastOnsetMs = 0;
  currentPhase = WARMUP;  phaseStartMs = millis();
  baselineIBICount = 0;  rmssdBaseline = 0.0f;  baselineValid = false;
  ibiCount = 0;  ibiWriteIdx = 0;  latestRMSSD = 0.0f;
  stressPPG = 0.0f;
  Serial.println("[RESET] 초기화 완료 → Warm-up 시작");
}

// ═════════════════════════════════════════════════════
// setup()
// ═════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(1500);
  Wire.begin();

  Serial.println("Initializing MAX30102...");
  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("[ERROR] MAX30102 not found");
    while (1);
  }
  sensor.setup(0x5F, 4, 2, 400, 411, 16384);
  Serial.println("[OK] MAX30102 초기화 완료");

  if (!BLE.begin()) {
    Serial.println("[ERROR] BLE 초기화 실패");
    while (1);
  }
  BLE.setLocalName("PPG_Stress_Monitor");
  BLE.setAdvertisedService(ppgService);
  ppgService.addCharacteristic(stressPacketChar);
  BLE.addService(ppgService);
  BLE.advertise();
  Serial.println("[OK] BLE 광고 시작 - PPG_Stress_Monitor");

  Serial.println("=== PPG v2 (버그수정) ===");
  Serial.println("time_ms,ir_raw,dc_blocked,smoothed,threshold,ibi_ms,rmssd,stress_ppg,phase");

  lastBLEMs    = millis();
  phaseStartMs = millis();
}

// ═════════════════════════════════════════════════════
// loop()
// ═════════════════════════════════════════════════════
void loop() {
  BLE.poll();

  unsigned long nowMs = millis();

  // ── [단계 1] 비블로킹 센서 읽기 ──────────────────
  // [버그 수정] getIR() → check()+available()+nextSample()
  // getIR()의 safeCheck(250): 데이터 없을 시 250ms 블로킹 후 0 반환
  // → 0 < 50000 판정으로 "손가락 떨어짐" 오검출
  // check()는 비블로킹: FIFO에 데이터 없으면 즉시 false 반환
  sensor.check();

  while (sensor.available()) {
    long irRaw = sensor.getIR();
    sensor.nextSample();
    nowMs = millis();

    // 손가락 감지
    bool nowFinger = (irRaw > FINGER_THRESHOLD);
    if (nowFinger != fingerDetected) {
      fingerDetected = nowFinger;
      if (fingerDetected) {
        Serial.println("[INFO] 손가락 감지 → 측정 시작");
        resetFilters();
      } else {
        Serial.println("[INFO] 손가락 떨어짐 → 측정 중지");
      }
    }
    if (!fingerDetected) continue;

    float curSample = (float)irRaw;

    // 단계 2~4
    float dcBlocked = applyDCBlocker(curSample);
    float smoothed  = applyMovingAverage(dcBlocked);
    updateThreshold(smoothed);

    // 구간 전환
    if (currentPhase == WARMUP && (nowMs - phaseStartMs) >= WARMUP_MS) {
      currentPhase = BASELINE;
      phaseStartMs = nowMs;
      Serial.println("[PHASE] Warm-up 완료 → Baseline 시작");
    }
    else if (currentPhase == BASELINE && (nowMs - phaseStartMs) >= BASELINE_MS) {
      float ibiSD = computeIBIStdDev(baselineIBIBuf, baselineIBICount);
      Serial.print("[BASELINE] IBI SD=");
      Serial.print(ibiSD, 2);
      Serial.println(" ms");
      if (ibiSD > BASELINE_IBI_SD_LIMIT || baselineIBICount < 10) {
        resetBaseline();
        phaseStartMs = nowMs;
      } else {
        rmssdBaseline = latestRMSSD;
        baselineValid = true;
        currentPhase  = MEASURING;
        phaseStartMs  = nowMs;
        Serial.print("[PHASE] Baseline 완료 → 실측 시작 | RMSSD_baseline=");
        Serial.print(rmssdBaseline, 2);
        Serial.println(" ms");
      }
    }

    // Onset 검출
    bool  nowAbove = (smoothed >= dynThresh);
    float ibi_ms   = 0.0f;
    bool  ibiValid = false;

    if (wasBelow && nowAbove) {
      bool refractOk = firstOnset || (nowMs - lastOnsetMs) >= REFRACTORY_MS;
      if (refractOk) {
        if (!firstOnset) {
          ibi_ms = (float)(nowMs - lastOnsetMs);
          if (ibi_ms >= IBI_MIN_MS && ibi_ms <= IBI_MAX_MS) {
            bool changeOk = true;
            if (ibiCount > 0) {
              float prevIBI = ibiBuf[(ibiWriteIdx - 1 + IBI_WINDOW) % IBI_WINDOW];
              float cr = abs(ibi_ms - prevIBI) / prevIBI;
              if (cr > IBI_CHANGE_LIMIT) {
                changeOk = false;
                Serial.print("[SKIP] 변화량 초과: ");
                Serial.print(ibi_ms, 1);
                Serial.print(" → 이전: ");
                Serial.println(prevIBI, 1);
              }
            }
            if (changeOk && currentPhase != WARMUP) {
              ibiValid = true;
              addIBI(ibi_ms);
              latestRMSSD = computeRMSSD();
              if (currentPhase == BASELINE && baselineIBICount < BASELINE_IBI_MAX)
                baselineIBIBuf[baselineIBICount++] = ibi_ms;
              if (currentPhase == MEASURING && baselineValid)
                stressPPG = computeStressPPG(latestRMSSD);
              Serial.print("[IBI] ");
              Serial.print(ibi_ms, 1);
              Serial.print(" ms | HR≈");
              Serial.print(60000.0f / ibi_ms, 1);
              Serial.print(" bpm | RMSSD=");
              Serial.print(latestRMSSD, 2);
              Serial.print(" ms | stress_PPG=");
              Serial.println(stressPPG, 4);
            }
          }
        }
        lastOnsetMs = nowMs;
        firstOnset  = false;
      }
    }
    wasBelow = !nowAbove;

    // 시리얼 플로터
    const char* ph =
      (currentPhase == WARMUP) ? "WARMUP" :
      (currentPhase == BASELINE) ? "BASELINE" : "MEASURING";
    Serial.print(nowMs);                   Serial.print(",");
    Serial.print(curSample / 1000.0f);     Serial.print(",");
    Serial.print(dcBlocked);              Serial.print(",");
    Serial.print(smoothed);               Serial.print(",");
    Serial.print(dynThresh);              Serial.print(",");
    Serial.print(ibiValid ? ibi_ms : 0.0f); Serial.print(",");
    Serial.print(latestRMSSD, 2);         Serial.print(",");
    Serial.print(stressPPG, 4);           Serial.print(",");
    Serial.println(ph);
  }

  // ── [단계 8] BLE 1초 고정 주기 ──────────────────
  // 손가락 미감지 시 송신 및 출력 완전 중단
  if (fingerDetected && (nowMs - lastBLEMs >= BLE_INTERVAL_MS)) {
    lastBLEMs += BLE_INTERVAL_MS;
    if (currentPhase == MEASURING && baselineValid) {
      sendBLEPacket(stressPPG, latestRMSSD, 0);
      Serial.print("[BLE] Stress=");
      Serial.print(stressPPG, 4);
      Serial.print(" | RMSSD=");
      Serial.print(latestRMSSD, 2);
      Serial.println(" ms");
    } else {
      Serial.print("[BLE] 대기 - ");
      Serial.println((currentPhase == WARMUP) ? "WARMUP" : "BASELINE");
    }
  }
}
