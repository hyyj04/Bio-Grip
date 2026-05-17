#include <Wire.h>
#include "MAX30105.h"
#include <ArduinoBLE.h>

MAX30105 particleSensor;

// ===============================
// BLE 설정
// ===============================
BLEService bioDataService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic featureDataChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 20);
BLEUnsignedCharCharacteristic controlPointChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLEWrite);

// BLE로 전송할 최종 특징 데이터
// 20 byte 제한을 맞추기 위해 핵심 결과만 바이너리 구조체로 전송한다.
#pragma pack(1)
struct BleFeatureData {
  float mean_hr;                 // 4 byte
  float final_ppg_stress_score;  // 4 byte
  float final_gsr_stress_score;  // 4 byte
  float integrated_stress_score; // 4 byte
  uint8_t integrated_level;      // 0: none, 1: Normal, 2: Mild, 3: Stress
  uint8_t mode_code;             // 0~6
  uint8_t flags;                 // bit0: ppgReady, bit1: gsrReady, bit2: labelValid, bit3: measuring
}; // 총 19 byte
#pragma pack()

BleFeatureData latestBleFeatures;

bool measurementActive = false;

float latestIntegratedStressScore = NAN;
String latestIntegratedStressLevel = "";
bool latestPpgReadyForBle = false;
bool latestGsrReadyForBle = false;

void startMeasurement();


namespace gsr {
/*
  GSR Signal Processing for Arduino IDE

  Version:
  Dual Baseline + Contact Check + Z-score + Stress Score + Rise Check

  Pipeline:
  1. Data Acquisition: 10 Hz
  2. Contact validity check
  3. Median Filter, N = 3
  4. SCL baseline: Moving Average, N = 150
  5. SCR detection baseline: EMA
  6. Phasic = medianGSR - scrDetectBaseline
  7. Common timeline: 0~15 sec warm-up, 15~75 sec baseline
  8. Measurement: after 75 sec
  9. 30 sec feature extraction
  10. Baseline-based z-score
  11. GSR stress score
*/

#define GSR_PIN A0

// =====================
// CSV output mode
// =====================
const bool PRINT_EVERY_SAMPLE = false;

// =====================
// Sampling parameters
// =====================
const unsigned long SAMPLE_INTERVAL_MS = 100;
const int FS_GSR = 10;

// =====================
// Contact validity parameters
// =====================
const int MIN_VALID_GSR_ADC = 10;
const int CONTACT_RECOVERY_SAMPLES = 10;
int contactRecoveryCount = 0;

// =====================
// Preprocessing parameters
// =====================
const int MEDIAN_N = 3;
const int MA_N = 150;     // 15 sec at 10 Hz

// SCR 검출용 EMA baseline
const float SCR_BASELINE_ALPHA = 0.02;

// =====================
// Time section parameters
// =====================
const int USER_WARMUP_SEC = 15;
const int BASELINE_DURATION_SEC = 60;
const int BASELINE_END_SEC = USER_WARMUP_SEC + BASELINE_DURATION_SEC;
const int FEATURE_WINDOW_SEC = 30;

const int MA_FILL_SAMPLES = MA_N;
const int USER_WARMUP_SAMPLES = USER_WARMUP_SEC * FS_GSR;

const int WARMUP_SAMPLES =
  (USER_WARMUP_SAMPLES > MA_FILL_SAMPLES) ? USER_WARMUP_SAMPLES : MA_FILL_SAMPLES;

const int BASELINE_END_SAMPLES  = BASELINE_END_SEC * FS_GSR;           // 750 samples (75s)
const int REST_END_SAMPLES       = BASELINE_END_SAMPLES + 300 * FS_GSR; // 3750 samples (375s)
const int TRANSITION_END_SAMPLES = REST_END_SAMPLES + 30 * FS_GSR;     // 4050 samples (405s)
const int STRESS_END_SAMPLES     = TRANSITION_END_SAMPLES + 270 * FS_GSR; // 6750 samples (675s)
const int FEATURE_WINDOW_SAMPLES = FEATURE_WINDOW_SEC * FS_GSR;

// =====================
// SCR detection parameters
// =====================
const float THRESHOLD_K = 2.5;
const float MIN_SCR_THRESHOLD_ADC = 6.0;
// threshold를 넘는 작은 흔들림을 SCR로 오검출하지 않도록,
// peak 후보가 직전 샘플 대비 최소한 이 정도 이상 상승했을 때만 SCR로 인정한다.
const float MIN_SCR_RISE_ADC = 2.0;
const float MAX_SCR_THRESHOLD_ADC = 15.0;
const int REFRACTORY_SAMPLES = 10;

// =====================
// Z-score / score parameters
// =====================
// baseline std가 너무 작을 때 z-score가 폭주하는 것을 방지하기 위한 최소 표준편차
const float MIN_SCL_STD_FOR_Z = 1.0;
const float MIN_PHASIC_STD_FOR_Z = 1.0;
const float MIN_FREQ_STD_FOR_Z = 2.0;

// z-score를 점수화할 때 사용할 scale
// z=0 -> 50점, z=+1 -> 65점, z=+2 -> 80점, z=+3 -> 95점
const float Z_SCORE_SCALE = 15.0;

// =====================
// Median filter variables
// =====================
float medBuf[MEDIAN_N] = {0, 0, 0};
int medIndex = 0;
bool medFilled = false;

// =====================
// SCL Moving average variables
// =====================
float maBuf[MA_N];
int maIndex = 0;
int maCount = 0;
float maSum = 0.0;

// =====================
// SCR EMA baseline variables
// =====================
float scrDetectBaseline = 0.0;
bool scrBaselineInitialized = false;

// =====================
// Last valid signal values
// =====================
float lastMedianGsr = 0.0;
float lastSclBaseline = 0.0;
float lastScrDetectBaseline = 0.0;
bool hasValidSignal = false;

// =====================
// Baseline statistics for threshold
// phasic 기준
// =====================
float baselinePhasicSum = 0.0;
float baselinePhasicSqSum = 0.0;
int baselineCount = 0;

float baselinePhasicMean = 0.0;
float baselinePhasicStd = 0.0;

// =====================
// Baseline statistics for z-score
// =====================
float baselineSclSum = 0.0;
float baselineSclSqSum = 0.0;
int baselineSclCount = 0;

float baselineSclMean = 0.0;
float baselineSclStd = 0.0;

float baselinePhasicPosSum = 0.0;
float baselinePhasicPosSqSum = 0.0;
int baselinePhasicPosCount = 0;

float baselinePhasicPosMean = 0.0;
float baselinePhasicPosStd = 0.0;

// NS-SCR Frequency는 baseline 안정 상태에서 0에 가깝게 보는 구조
float baselineNsScrFreqMean = 0.0;
float baselineNsScrFreqStd = MIN_FREQ_STD_FOR_Z;

// =====================
// Threshold
// =====================
float scrThreshold = MIN_SCR_THRESHOLD_ADC;
bool baselineReady = false;

// =====================
// Feature window variables
// =====================
float windowSclSum = 0.0;
int windowSampleCount = 0;

float windowScrAmpSum = 0.0;
int windowScrCount = 0;

// =====================
// SCR peak detection variables
// =====================
float phasicPrev2 = 0.0;
float phasicPrev1 = 0.0;
float phasicCurr = 0.0;

int lastScrSampleIndex = -9999;

int detectedScrSampleIndex = -1;
unsigned long detectedScrTimeMs = 0;

// =====================
// Sample time variables
// =====================
unsigned long prev2SampleTimeMs = 0;
unsigned long prev1SampleTimeMs = 0;
unsigned long currSampleTimeMs = 0;

// =====================
// General variables
// =====================
unsigned long lastSampleTime = 0;
int sampleIndex = 0;

// =====================
// Latest GSR values for integrated output
// =====================
String latestMode = "STABILIZING";
int latestRawGsr = 0;
int latestSampleIndex = 0;
int latestContactOk = 0;
int latestValidSampleFlag = 0;
float latestMedianGsr = NAN;
float latestSclBaseline = NAN;
float latestScrDetectBaseline = NAN;
float latestPhasic = NAN;
float latestPhasicPos = NAN;
float latestSclMean30s = NAN;
float latestScrAmplitudeMean30s = NAN;
float latestNsScrFrequencyPerMin = NAN;
float latestSclMeanZ = NAN;
float latestScrAmplitudeMeanZ = NAN;
float latestNsScrFrequencyZ = NAN;
float latestSclStressScore = NAN;
float latestScrAmplitudeStressScore = NAN;
float latestNsScrFrequencyStressScore = NAN;
float latestFinalGsrStressScore = NAN;
int latestFeatureReady = 0;

// GSR feature는 30초 window 단위로 갱신되므로, 최근 계산값을 일정 시간 동안만 유효하게 사용한다.
// 이렇게 하면 새 window가 완성되기 전 출력 공백은 줄이고, 너무 오래된 값이 현재 상태처럼 통합되는 문제는 막을 수 있다.
const unsigned long GSR_SCORE_FRESHNESS_MS = (FEATURE_WINDOW_SEC + 10UL) * 1000UL;
unsigned long latestGsrScoreUpdateMs = 0;

bool isLatestScoreFresh(unsigned long nowMs) {
  if (latestGsrScoreUpdateMs == 0) return false;
  return (nowMs - latestGsrScoreUpdateMs) <= GSR_SCORE_FRESHNESS_MS;
}


// =====================
// Utility functions
// =====================
float median3(float a, float b, float c) {
  if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
  if ((b >= a && b <= c) || (b <= a && b >= c)) return b;
  return c;
}

float clampFloat(float x, float low, float high) {
  if (x < low) return low;
  if (x > high) return high;
  return x;
}

float safeStd(float stdVal, float minStd) {
  if (stdVal < minStd) return minStd;
  return stdVal;
}

float computeZScore(float value, float meanVal, float stdVal, float minStd) {
  float s = safeStd(stdVal, minStd);
  return (value - meanVal) / s;
}

float zToStressScore(float z) {
  float zClamped = clampFloat(z, -3.0, 3.0);
  float score = 50.0 + (zClamped * Z_SCORE_SCALE);
  return clampFloat(score, 0.0, 100.0);
}

float calcMean(float sumVal, int countVal) {
  if (countVal <= 0) return 0.0;
  return sumVal / countVal;
}

float calcStd(float sumVal, float sqSumVal, int countVal) {
  if (countVal <= 1) return 0.0;

  float meanVal = sumVal / countVal;
  float variance = (sqSumVal / countVal) - (meanVal * meanVal);

  if (variance < 0.0) {
    variance = 0.0;
  }

  return sqrt(variance);
}


// =====================
// Median filter update
// =====================
float updateMedianFilter(float x) {
  medBuf[medIndex] = x;
  medIndex++;

  if (medIndex >= MEDIAN_N) {
    medIndex = 0;
    medFilled = true;
  }

  if (!medFilled) {
    return x;
  }

  return median3(medBuf[0], medBuf[1], medBuf[2]);
}


// =====================
// SCL Moving Average update
// =====================
float updateMovingAverage(float x) {
  if (maCount < MA_N) {
    maBuf[maIndex] = x;
    maSum += x;
    maCount++;
  } else {
    maSum -= maBuf[maIndex];
    maBuf[maIndex] = x;
    maSum += x;
  }

  maIndex++;
  if (maIndex >= MA_N) {
    maIndex = 0;
  }

  return maSum / maCount;
}


// =====================
// SCR EMA baseline update
// =====================
float updateEmaBaseline(float x) {
  if (!scrBaselineInitialized) {
    scrDetectBaseline = x;
    scrBaselineInitialized = true;
  } else {
    scrDetectBaseline =
      SCR_BASELINE_ALPHA * x +
      (1.0 - SCR_BASELINE_ALPHA) * scrDetectBaseline;
  }

  return scrDetectBaseline;
}


// =====================
// Baseline finalization
// =====================
void finalizeBaseline() {
  baselinePhasicMean = calcMean(baselinePhasicSum, baselineCount);
  baselinePhasicStd = calcStd(baselinePhasicSum, baselinePhasicSqSum, baselineCount);

  baselineSclMean = calcMean(baselineSclSum, baselineSclCount);
  baselineSclStd = calcStd(baselineSclSum, baselineSclSqSum, baselineSclCount);

  baselinePhasicPosMean = calcMean(baselinePhasicPosSum, baselinePhasicPosCount);
  baselinePhasicPosStd = calcStd(baselinePhasicPosSum, baselinePhasicPosSqSum, baselinePhasicPosCount);

  // SCR detection threshold
  scrThreshold = THRESHOLD_K * baselinePhasicStd;

  if (scrThreshold < MIN_SCR_THRESHOLD_ADC) {
    scrThreshold = MIN_SCR_THRESHOLD_ADC;
  }

  if (scrThreshold > MAX_SCR_THRESHOLD_ADC) {
    scrThreshold = MAX_SCR_THRESHOLD_ADC;
  }

  // Frequency baseline은 안정 상태에서 0회/min으로 간주
  baselineNsScrFreqMean = 0.0;
  baselineNsScrFreqStd = MIN_FREQ_STD_FOR_Z;

  baselineReady = true;
}


// =====================
// Peak detection buffer reset
// =====================
void resetPeakDetectionBuffer() {
  phasicPrev2 = 0.0;
  phasicPrev1 = 0.0;
  phasicCurr = 0.0;
  lastScrSampleIndex = sampleIndex;
  detectedScrSampleIndex = -1;
  detectedScrTimeMs = 0;
}


// =====================
// CSV header
// =====================
void printHeader() {
  Serial.println(
    "time_ms,"
    "sample_index,"
    "mode,"
    "raw_gsr,"
    "median_gsr,"
    "scl_baseline,"
    "scr_detect_baseline,"
    "phasic,"
    "phasic_pos,"
    "scr_threshold,"
    "scr_flag,"
    "scr_peak_time_ms,"
    "scr_peak_sample_index,"
    "scl_mean_30s,"
    "scr_amplitude_mean_30s,"
    "ns_scr_frequency_per_min,"
    "baseline_scl_mean,"
    "baseline_scl_std,"
    "baseline_phasic_mean,"
    "baseline_phasic_std,"
    "baseline_phasic_pos_mean,"
    "baseline_phasic_pos_std,"
    "scl_mean_z,"
    "scr_amplitude_mean_z,"
    "ns_scr_frequency_z,"
    "scl_stress_score,"
    "scr_amplitude_stress_score,"
    "ns_scr_frequency_stress_score,"
    "final_gsr_stress_score,"
    "feature_ready,"
    "min_scr_threshold_adc,"
    "max_scr_threshold_adc,"
    "contact_ok,"
    "valid_sample_flag"
  );
}


// =====================
// CSV row print
// =====================
void printCsvRow(
  unsigned long timeMs,
  int sIndex,
  String mode,
  int rawGsr,
  float medianGsr,
  float sclBaseline,
  float scrDetectBl,
  float phasic,
  float phasicPos,
  int scrFlag,
  unsigned long scrPeakTimeMs,
  int scrPeakSampleIndex,
  float sclMean30s,
  float scrAmplitudeMean30s,
  float nsScrFrequencyPerMin,
  float sclMeanZ,
  float scrAmplitudeMeanZ,
  float nsScrFrequencyZ,
  float sclStressScore,
  float scrAmplitudeStressScore,
  float nsScrFrequencyStressScore,
  float finalGsrStressScore,
  int featureReady,
  int contactOk,
  int validSampleFlag
) {
  // Suppressed in integrated version. Unified CSV is printed by the main PPG loop.
}



// =====================
// Setup
// =====================
void setup() {
  for (int i = 0; i < MA_N; i++) {
    maBuf[i] = 0.0;
  }
}


// =====================
// Reset all GSR variables
// =====================
void resetAll() {
  // Median filter
  for (int i = 0; i < MEDIAN_N; i++) medBuf[i] = 0.0;
  medIndex = 0;
  medFilled = false;

  // MA buffer
  for (int i = 0; i < MA_N; i++) maBuf[i] = 0.0;
  maIndex = 0;
  maCount = 0;
  maSum = 0.0;

  // EMA baseline
  scrDetectBaseline = 0.0;
  scrBaselineInitialized = false;

  // Last valid signal
  lastMedianGsr = 0.0;
  lastSclBaseline = 0.0;
  lastScrDetectBaseline = 0.0;
  hasValidSignal = false;

  // Baseline statistics
  baselinePhasicSum = 0.0;
  baselinePhasicSqSum = 0.0;
  baselineCount = 0;
  baselinePhasicMean = 0.0;
  baselinePhasicStd = 0.0;

  baselineSclSum = 0.0;
  baselineSclSqSum = 0.0;
  baselineSclCount = 0;
  baselineSclMean = 0.0;
  baselineSclStd = 0.0;

  baselinePhasicPosSum = 0.0;
  baselinePhasicPosSqSum = 0.0;
  baselinePhasicPosCount = 0;
  baselinePhasicPosMean = 0.0;
  baselinePhasicPosStd = 0.0;

  baselineNsScrFreqMean = 0.0;
  baselineNsScrFreqStd = MIN_FREQ_STD_FOR_Z;

  // Threshold & ready flag
  scrThreshold = MIN_SCR_THRESHOLD_ADC;
  baselineReady = false;

  // Feature window
  windowSclSum = 0.0;
  windowSampleCount = 0;
  windowScrAmpSum = 0.0;
  windowScrCount = 0;

  // SCR peak detection
  phasicPrev2 = 0.0;
  phasicPrev1 = 0.0;
  phasicCurr = 0.0;
  lastScrSampleIndex = -9999;
  detectedScrSampleIndex = -1;
  detectedScrTimeMs = 0;

  // Sample time
  prev2SampleTimeMs = 0;
  prev1SampleTimeMs = 0;
  currSampleTimeMs = 0;

  // General
  sampleIndex = 0;
  contactRecoveryCount = 0;

  // Latest output values
  latestMode = "STABILIZING";
  latestRawGsr = 0;
  latestSampleIndex = 0;
  latestContactOk = 0;
  latestValidSampleFlag = 0;
  latestMedianGsr = NAN;
  latestSclBaseline = NAN;
  latestScrDetectBaseline = NAN;
  latestPhasic = NAN;
  latestPhasicPos = NAN;
  latestSclMean30s = NAN;
  latestScrAmplitudeMean30s = NAN;
  latestNsScrFrequencyPerMin = NAN;
  latestSclMeanZ = NAN;
  latestScrAmplitudeMeanZ = NAN;
  latestNsScrFrequencyZ = NAN;
  latestSclStressScore = NAN;
  latestScrAmplitudeStressScore = NAN;
  latestNsScrFrequencyStressScore = NAN;
  latestFinalGsrStressScore = NAN;
  latestFeatureReady = 0;
  latestGsrScoreUpdateMs = 0;
}


// =====================
// Main loop
// =====================
void loop() {
  unsigned long now = millis();

  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) {
    return;
  }

  lastSampleTime = now;

  prev2SampleTimeMs = prev1SampleTimeMs;
  prev1SampleTimeMs = currSampleTimeMs;
  currSampleTimeMs = now;

  String mode;

  if (sampleIndex < WARMUP_SAMPLES) {
    mode = "STABILIZING";
  } else if (sampleIndex < BASELINE_END_SAMPLES) {
    mode = "CALIBRATION";
  } else if (sampleIndex < REST_END_SAMPLES) {
    mode = "REST";
  } else if (sampleIndex < TRANSITION_END_SAMPLES) {
    mode = "TRANSITION";
  } else if (sampleIndex < STRESS_END_SAMPLES) {
    mode = "STRESS";
  } else {
    mode = "DONE";
  }

  int rawGsr = analogRead(GSR_PIN);

  bool contactOkBool = rawGsr >= MIN_VALID_GSR_ADC;
  int contactOk = contactOkBool ? 1 : 0;

  latestMode = mode;
  latestRawGsr = rawGsr;
  latestSampleIndex = sampleIndex;
  latestContactOk = contactOk;

  float medianGsr = lastMedianGsr;
  float sclBaseline = lastSclBaseline;
  float scrDetectBl = lastScrDetectBaseline;

  float phasic = 0.0;
  float phasicPos = 0.0;

  int scrFlag = 0;
  detectedScrSampleIndex = -1;
  detectedScrTimeMs = 0;

  float sclMean30s = 0.0;
  float scrAmplitudeMean30s = 0.0;
  float nsScrFrequencyPerMin = 0.0;

  float sclMeanZ = 0.0;
  float scrAmplitudeMeanZ = 0.0;
  float nsScrFrequencyZ = 0.0;

  float sclStressScore = 0.0;
  float scrAmplitudeStressScore = 0.0;
  float nsScrFrequencyStressScore = 0.0;
  float finalGsrStressScore = 0.0;

  int featureReady = 0;

  // baseline 종료 시점 보장
  if (!baselineReady && sampleIndex >= BASELINE_END_SAMPLES) {
    finalizeBaseline();
    resetPeakDetectionBuffer();
  }

  // 접촉 불량 샘플 처리
  if (!contactOkBool) {
    contactRecoveryCount = CONTACT_RECOVERY_SAMPLES;

    if (!hasValidSignal) {
      medianGsr = 0.0;
      sclBaseline = 0.0;
      scrDetectBl = 0.0;
    }

    latestMedianGsr = medianGsr;
    latestSclBaseline = sclBaseline;
    latestScrDetectBaseline = scrDetectBl;
    latestPhasic = phasic;
    latestPhasicPos = phasicPos;
    latestValidSampleFlag = 0;
    latestFeatureReady = 0;

    // 접촉 불량 구간에서도 최근 GSR feature 값은 즉시 지우지 않는다.
    // 통합 단계에서 contact_ok와 freshness time으로 사용 여부를 판단한다.

    if (PRINT_EVERY_SAMPLE) {
      printCsvRow(
        now,
        sampleIndex,
        mode,
        rawGsr,
        medianGsr,
        sclBaseline,
        scrDetectBl,
        phasic,
        phasicPos,
        scrFlag,
        detectedScrTimeMs,
        detectedScrSampleIndex,
        sclMean30s,
        scrAmplitudeMean30s,
        nsScrFrequencyPerMin,
        sclMeanZ,
        scrAmplitudeMeanZ,
        nsScrFrequencyZ,
        sclStressScore,
        scrAmplitudeStressScore,
        nsScrFrequencyStressScore,
        finalGsrStressScore,
        featureReady,
        contactOk,
        0
      );
    }

    sampleIndex++;
    return;
  }

  // Median filter
  medianGsr = updateMedianFilter((float)rawGsr);

  // Dual baseline
  sclBaseline = updateMovingAverage(medianGsr);
  scrDetectBl = updateEmaBaseline(medianGsr);

  lastMedianGsr = medianGsr;
  lastSclBaseline = sclBaseline;
  lastScrDetectBaseline = scrDetectBl;
  hasValidSignal = true;

  int validSampleFlag = 1;

  if (contactRecoveryCount > 0) {
    contactRecoveryCount--;
    validSampleFlag = 0;
  }

  // Phasic
  phasic = medianGsr - scrDetectBl;

  phasicPos = phasic;
  if (phasicPos < 0.0) {
    phasicPos = 0.0;
  }

  latestMedianGsr = medianGsr;
  latestSclBaseline = sclBaseline;
  latestScrDetectBaseline = scrDetectBl;
  latestPhasic = phasic;
  latestPhasicPos = phasicPos;
  latestValidSampleFlag = validSampleFlag;

  // 새 30초 feature window가 완성되지 않은 샘플에서도, 최근 GSR 점수가 freshness time 안에 있으면 유효 feature로 표시한다.
  latestFeatureReady = isLatestScoreFresh(now) ? 1 : 0;

  // baseline/monitoring 전환 전에도 최근 feature 값을 강제로 지우지 않는다.
  // latestGsrScoreUpdateMs가 0이면 freshness check에서 자동으로 제외된다.

  // Baseline statistics
  if (mode == "CALIBRATION" && validSampleFlag == 1) {
    baselinePhasicSum += phasic;
    baselinePhasicSqSum += phasic * phasic;
    baselineCount++;

    baselineSclSum += sclBaseline;
    baselineSclSqSum += sclBaseline * sclBaseline;
    baselineSclCount++;

    baselinePhasicPosSum += phasicPos;
    baselinePhasicPosSqSum += phasicPos * phasicPos;
    baselinePhasicPosCount++;
  }

  // SCR peak detection
  if ((mode == "REST" || mode == "STRESS") && baselineReady && validSampleFlag == 1) {
    phasicPrev2 = phasicPrev1;
    phasicPrev1 = phasicCurr;
    phasicCurr = phasicPos;

    bool localPeak =
      (phasicPrev1 > phasicPrev2) &&
      (phasicPrev1 >= phasicCurr);

    bool aboveThreshold =
      (phasicPrev1 >= scrThreshold) &&
      (phasicPrev1 > 0.0);

    bool refractoryOk =
      ((sampleIndex - 1) - lastScrSampleIndex) >= REFRACTORY_SAMPLES;

    // local peak가 threshold를 넘더라도, 직전 샘플 대비 상승량이 너무 작으면
    // 접촉 잡음/미세 흔들림일 가능성이 있어 SCR로 인정하지 않는다.
    bool riseOk =
      (phasicPrev1 - phasicPrev2) >= MIN_SCR_RISE_ADC;

    if (localPeak && aboveThreshold && refractoryOk && riseOk) {
      scrFlag = 1;

      detectedScrSampleIndex = sampleIndex - 1;
      detectedScrTimeMs = prev1SampleTimeMs;

      lastScrSampleIndex = detectedScrSampleIndex;

      windowScrCount++;
      windowScrAmpSum += phasicPrev1;
    }
  } else {
    phasicPrev2 = 0.0;
    phasicPrev1 = 0.0;
    phasicCurr = 0.0;
  }

  // Feature window
  if ((mode == "REST" || mode == "STRESS") && baselineReady && validSampleFlag == 1) {
    windowSclSum += sclBaseline;
    windowSampleCount++;

    if (windowSampleCount >= FEATURE_WINDOW_SAMPLES) {
      featureReady = 1;

      sclMean30s = windowSclSum / windowSampleCount;

      if (windowScrCount > 0) {
        scrAmplitudeMean30s = windowScrAmpSum / windowScrCount;
      } else {
        scrAmplitudeMean30s = 0.0;
      }

      nsScrFrequencyPerMin =
        windowScrCount * (60.0 / FEATURE_WINDOW_SEC);

      // =====================
      // Z-score 계산
      // =====================
      sclMeanZ = computeZScore(
        sclMean30s,
        baselineSclMean,
        baselineSclStd,
        MIN_SCL_STD_FOR_Z
      );

      scrAmplitudeMeanZ = computeZScore(
        scrAmplitudeMean30s,
        baselinePhasicPosMean,
        baselinePhasicPosStd,
        MIN_PHASIC_STD_FOR_Z
      );

      nsScrFrequencyZ = computeZScore(
        nsScrFrequencyPerMin,
        baselineNsScrFreqMean,
        baselineNsScrFreqStd,
        MIN_FREQ_STD_FOR_Z
      );

      // =====================
      // Stress score 계산
      // =====================
      sclStressScore = zToStressScore(sclMeanZ);
      scrAmplitudeStressScore = zToStressScore(scrAmplitudeMeanZ);
      nsScrFrequencyStressScore = zToStressScore(nsScrFrequencyZ);

      // GSR 최종 점수는 임의 가중치를 두지 않고, 유효한 지표만 조건부 평균한다.
      // SCL은 30초 평균이 항상 존재하므로 기본 지표로 사용한다.
      // SCR amplitude와 NS-SCR frequency는 해당 window에서 SCR이 실제로 검출된 경우에만 평균에 포함한다.
      float gsrScoreSum = 0.0;
      int gsrScoreCount = 0;

      if (!isnan(sclStressScore)) {
        gsrScoreSum += sclStressScore;
        gsrScoreCount++;
      }

      if (windowScrCount > 0 && !isnan(scrAmplitudeStressScore)) {
        gsrScoreSum += scrAmplitudeStressScore;
        gsrScoreCount++;
      }

      if (windowScrCount > 0 && !isnan(nsScrFrequencyStressScore)) {
        gsrScoreSum += nsScrFrequencyStressScore;
        gsrScoreCount++;
      }

      if (gsrScoreCount > 0) {
        finalGsrStressScore = gsrScoreSum / gsrScoreCount;
      } else {
        finalGsrStressScore = NAN;
      }

      latestSclMean30s = sclMean30s;
      latestScrAmplitudeMean30s = scrAmplitudeMean30s;
      latestNsScrFrequencyPerMin = nsScrFrequencyPerMin;
      latestSclMeanZ = sclMeanZ;
      latestScrAmplitudeMeanZ = scrAmplitudeMeanZ;
      latestNsScrFrequencyZ = nsScrFrequencyZ;
      latestSclStressScore = sclStressScore;
      latestScrAmplitudeStressScore = scrAmplitudeStressScore;
      latestNsScrFrequencyStressScore = nsScrFrequencyStressScore;
      latestFinalGsrStressScore = finalGsrStressScore;
      latestFeatureReady = 1;
      latestGsrScoreUpdateMs = now;

      printCsvRow(
        now,
        sampleIndex,
        mode,
        rawGsr,
        medianGsr,
        sclBaseline,
        scrDetectBl,
        phasic,
        phasicPos,
        scrFlag,
        detectedScrTimeMs,
        detectedScrSampleIndex,
        sclMean30s,
        scrAmplitudeMean30s,
        nsScrFrequencyPerMin,
        sclMeanZ,
        scrAmplitudeMeanZ,
        nsScrFrequencyZ,
        sclStressScore,
        scrAmplitudeStressScore,
        nsScrFrequencyStressScore,
        finalGsrStressScore,
        featureReady,
        contactOk,
        validSampleFlag
      );

      windowSclSum = 0.0;
      windowSampleCount = 0;
      windowScrAmpSum = 0.0;
      windowScrCount = 0;

      sampleIndex++;
      return;
    }
  }

  if (PRINT_EVERY_SAMPLE) {
    printCsvRow(
      now,
      sampleIndex,
      mode,
      rawGsr,
      medianGsr,
      sclBaseline,
      scrDetectBl,
      phasic,
      phasicPos,
      scrFlag,
      detectedScrTimeMs,
      detectedScrSampleIndex,
      sclMean30s,
      scrAmplitudeMean30s,
      nsScrFrequencyPerMin,
      sclMeanZ,
      scrAmplitudeMeanZ,
      nsScrFrequencyZ,
      sclStressScore,
      scrAmplitudeStressScore,
      nsScrFrequencyStressScore,
      finalGsrStressScore,
      featureReady,
      contactOk,
      validSampleFlag
    );
  }

  sampleIndex++;
}
} // namespace gsr



// ===============================
// 1. 샘플링 및 구간 설정
// ===============================
const float FS = 100.0;
const unsigned long SAMPLE_INTERVAL_MS = 10;

const unsigned long STABILIZING_TIME_MS  = 15000;
const unsigned long CALIBRATION_TIME_MS  = 60000;
const unsigned long REST_TIME_MS         = 300000;  // 5분 안정 구간, label 0
const unsigned long TRANSITION_TIME_MS   = 30000;   // 스트레스 과제 시작 후 30초, 라벨링 제외
const unsigned long STRESS_TIME_MS       = 270000;  // 스트레스 후반 4분 30초, label 1
// 총 측정 시간: 15 + 60 + 300 + 30 + 270 = 675초 (11분 15초)

unsigned long startTime = 0;
unsigned long lastSampleTime = 0;
unsigned long lastPrintTime = 0;

// DONE 상태에서 완료 메시지를 한 번만 출력하고, R 명령으로 재시작할 수 있게 하는 플래그
bool measurementDonePrinted = false;

const unsigned long PRINT_INTERVAL_MS = 1000;


// ===============================
// 2. 손가락 미접촉 판단 설정
// ===============================
const long NO_FINGER_IR_THRESHOLD = 10000;

const int NO_FINGER_CONFIRM_COUNT = 10;
int consecutiveNoFinger = 0;

int noFingerCount = 0;


// ===============================
// 3. Butterworth 필터 계수
// HPF 0.5 Hz, LPF 8 Hz, Fs 100 Hz
// ===============================

// HPF
float b_hp[3] = {0.97803048, -1.95606096, 0.97803048};
float a_hp[3] = {1.0, -1.95557824, 0.95654368};

// LPF
float b_lp[3] = {0.0461318, 0.0922636, 0.0461318};
float a_lp[3] = {1.0, -1.30728503, 0.49181224};

float hp_x1 = 0, hp_x2 = 0;
float hp_y1 = 0, hp_y2 = 0;

float lp_x1 = 0, lp_x2 = 0;
float lp_y1 = 0, lp_y2 = 0;


// ===============================
// 4. Peak 검출 및 IBI 설정
// ===============================
float ppg_prev2 = 0;
float ppg_prev1 = 0;
float ppg_curr = 0;

float abs_ema = 0;
const float ABS_EMA_ALPHA = 0.01;
const float PEAK_THRESHOLD_RATIO = 0.35;

unsigned long lastPeakTime = 0;
const unsigned long REFRACTORY_MS = 300;

// rawIbiMs: peak 간격으로 계산된 원시 IBI
// validIbiMs: 제2 전처리까지 통과한 유효 IBI
float rawIbiMs = NAN;
float validIbiMs = NAN;

const float MIN_IBI_MS = 300.0;
const float MAX_IBI_MS = 1500.0;

// local median 기반 이상치 제거
const int MEDIAN_BUF_SIZE = 7;
float medianBuf[MEDIAN_BUF_SIZE];
int medianCount = 0;
int medianIndex = 0;

const float LOCAL_MEDIAN_TOLERANCE = 0.30;


// ===============================
// 5. HRV 계산용 IBI 버퍼
// ===============================
const int IBI_BUF_SIZE = 30;
float ibiBuf[IBI_BUF_SIZE];
int ibiCount = 0;
int ibiIndex = 0;

float meanHR = NAN;
float sdnn = NAN;
float rmssd = NAN;


// ===============================
// 6. Baseline HRV 계산용 변수
// ===============================
int baselineMetricCount = 0;

float baselineMeanHRMean = NAN;
float baselineMeanHRStd = NAN;

float baselineSDNNMean = NAN;
float baselineSDNNStd = NAN;

float baselineRMSSDMean = NAN;
float baselineRMSSDStd = NAN;

float meanHR_m = 0, meanHR_s = 0;
float sdnn_m = 0, sdnn_s = 0;
float rmssd_m = 0, rmssd_s = 0;


// ===============================
// 7. Baseline IBI 변동 범위 계산용 변수
// ===============================
int baselineIbiCount = 0;

float baselineIbiMean = NAN;
float baselineIbiStd = NAN;

float baselineIbi_m = 0;
float baselineIbi_s = 0;

// 마우스 기반 PPG 환경을 고려해 4SD로 완화
const float BASELINE_IBI_STD_MULT = 4.0;


// ===============================
// 8. SQI window 설정
// ===============================
const int SQI_BUF_SIZE = 10;
const int SQI_MIN_COUNT = 5;

float sqiIbiBuf[SQI_BUF_SIZE];
float sqiAmpBuf[SQI_BUF_SIZE];

int sqiCount = 0;
int sqiIndex = 0;

// 마우스 기반 PPG 환경을 고려해 완화한 SQI 기준
const float MAX_IBI_CV = 0.45;
const float MAX_AMP_CV = 1.20;
const float MIN_SQI_SCORE = 50.0;

float currentSQI = NAN;
int validWindowFlag = 0;


// ===============================
// 9. Z-score 기반 PPG 스트레스 점수 변수
// ===============================
float meanHrZ = NAN;
float sdnnZ = NAN;
float rmssdZ = NAN;

float meanHrStressScore = NAN;
float sdnnStressScore = NAN;
float rmssdStressScore = NAN;

float ppgAvgScore = NAN;
float hrvScore = NAN;
float finalPpgStressScore = NAN;

String finalPpgStressLevel = "";

const float MAX_Z_FOR_SCORE = 3.0;

const float NORMAL_THRESHOLD = 33.3;
const float STRESS_THRESHOLD = 66.7;

const int MIN_BASELINE_METRIC_COUNT_FOR_Z = 20;

// baseline 표준편차가 너무 작을 때 z-score가 과하게 튀는 문제 방지
const float MIN_MEAN_HR_STD_FOR_Z = 3.0;
const float MIN_SDNN_STD_FOR_Z = 5.0;
const float MIN_RMSSD_STD_FOR_Z = 5.0;


// ===============================
// 10. 출력 유틸 함수
// ===============================
void printFloatOrEmpty(float x, int digits) {
  if (isnan(x)) {
    Serial.print("");
  } else {
    Serial.print(x, digits);
  }
}


// ===============================
// 11. 필터 함수
// ===============================
float applyHPF(float x) {
  float y = b_hp[0] * x + b_hp[1] * hp_x1 + b_hp[2] * hp_x2
            - a_hp[1] * hp_y1 - a_hp[2] * hp_y2;

  hp_x2 = hp_x1;
  hp_x1 = x;
  hp_y2 = hp_y1;
  hp_y1 = y;

  return y;
}


float applyLPF(float x) {
  float y = b_lp[0] * x + b_lp[1] * lp_x1 + b_lp[2] * lp_x2
            - a_lp[1] * lp_y1 - a_lp[2] * lp_y2;

  lp_x2 = lp_x1;
  lp_x1 = x;
  lp_y2 = lp_y1;
  lp_y1 = y;

  return y;
}


float applyBandpass(float x) {
  float hp = applyHPF(x);
  float lp = applyLPF(hp);
  return lp;
}


// ===============================
// 12. 통계 함수
// ===============================
float calcMean(float arr[], int n) {
  if (n <= 0) return NAN;

  float sum = 0;
  for (int i = 0; i < n; i++) {
    sum += arr[i];
  }

  return sum / n;
}


float calcStd(float arr[], int n) {
  if (n < 2) return NAN;

  float m = calcMean(arr, n);
  float sumSq = 0;

  for (int i = 0; i < n; i++) {
    float d = arr[i] - m;
    sumSq += d * d;
  }

  return sqrt(sumSq / (n - 1));
}


float calcMedian(float arr[], int n) {
  if (n <= 0) return NAN;

  float temp[MEDIAN_BUF_SIZE];

  for (int i = 0; i < n; i++) {
    temp[i] = arr[i];
  }

  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (temp[j] < temp[i]) {
        float t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }

  if (n % 2 == 1) {
    return temp[n / 2];
  } else {
    return (temp[n / 2 - 1] + temp[n / 2]) / 2.0;
  }
}


float calcCV(float arr[], int n) {
  if (n < 2) return NAN;

  float meanValue = calcMean(arr, n);
  float stdValue = calcStd(arr, n);

  if (isnan(meanValue) || meanValue == 0) return NAN;
  if (isnan(stdValue)) return NAN;

  return stdValue / meanValue;
}


float clampFloat(float x, float low, float high) {
  if (x < low) return low;
  if (x > high) return high;
  return x;
}


// ===============================
// 13. Welford baseline 계산 함수
// ===============================
void updateWelford(float x, int n, float &m, float &s) {
  float delta = x - m;
  m += delta / n;
  float delta2 = x - m;
  s += delta * delta2;
}


void updateBaselineMetrics() {
  if (isnan(meanHR) || isnan(sdnn) || isnan(rmssd)) {
    return;
  }

  baselineMetricCount++;

  updateWelford(meanHR, baselineMetricCount, meanHR_m, meanHR_s);
  updateWelford(sdnn, baselineMetricCount, sdnn_m, sdnn_s);
  updateWelford(rmssd, baselineMetricCount, rmssd_m, rmssd_s);

  baselineMeanHRMean = meanHR_m;
  baselineSDNNMean = sdnn_m;
  baselineRMSSDMean = rmssd_m;

  if (baselineMetricCount >= 2) {
    baselineMeanHRStd = sqrt(meanHR_s / (baselineMetricCount - 1));
    baselineSDNNStd = sqrt(sdnn_s / (baselineMetricCount - 1));
    baselineRMSSDStd = sqrt(rmssd_s / (baselineMetricCount - 1));
  }
}


// baseline IBI는 basic IBI 검사를 통과한 CALIBRATION IBI로 먼저 형성
void updateBaselineIbi(float ibi) {
  if (isnan(ibi)) return;

  baselineIbiCount++;

  float delta = ibi - baselineIbi_m;
  baselineIbi_m += delta / baselineIbiCount;
  float delta2 = ibi - baselineIbi_m;
  baselineIbi_s += delta * delta2;

  baselineIbiMean = baselineIbi_m;

  if (baselineIbiCount >= 2) {
    baselineIbiStd = sqrt(baselineIbi_s / (baselineIbiCount - 1));
  }
}


// ===============================
// 14. 버퍼 함수
// ===============================
void addMedianBuffer(float ibi) {
  medianBuf[medianIndex] = ibi;
  medianIndex = (medianIndex + 1) % MEDIAN_BUF_SIZE;

  if (medianCount < MEDIAN_BUF_SIZE) {
    medianCount++;
  }
}


void addIbiBuffer(float ibi) {
  ibiBuf[ibiIndex] = ibi;
  ibiIndex = (ibiIndex + 1) % IBI_BUF_SIZE;

  if (ibiCount < IBI_BUF_SIZE) {
    ibiCount++;
  }
}


void addSqiBuffer(float ibi, float amp) {
  sqiIbiBuf[sqiIndex] = ibi;
  sqiAmpBuf[sqiIndex] = abs(amp);

  sqiIndex = (sqiIndex + 1) % SQI_BUF_SIZE;

  if (sqiCount < SQI_BUF_SIZE) {
    sqiCount++;
  }
}


// ===============================
// 15. IBI 이상치 검사
// ===============================
bool isBasicValidIbi(float ibi) {
  if (isnan(ibi)) return false;

  if (ibi < MIN_IBI_MS || ibi > MAX_IBI_MS) {
    return false;
  }

  if (medianCount >= 3) {
    float med = calcMedian(medianBuf, medianCount);
    float lower = med * (1.0 - LOCAL_MEDIAN_TOLERANCE);
    float upper = med * (1.0 + LOCAL_MEDIAN_TOLERANCE);

    if (ibi < lower || ibi > upper) {
      return false;
    }
  }

  return true;
}


bool isBaselineRangeValidIbi(float ibi) {
  if (
    baselineIbiCount >= 10 &&
    !isnan(baselineIbiMean) &&
    !isnan(baselineIbiStd) &&
    baselineIbiStd > 0
  ) {
    float baselineLower = baselineIbiMean - BASELINE_IBI_STD_MULT * baselineIbiStd;
    float baselineUpper = baselineIbiMean + BASELINE_IBI_STD_MULT * baselineIbiStd;

    if (baselineLower < MIN_IBI_MS) baselineLower = MIN_IBI_MS;
    if (baselineUpper > MAX_IBI_MS) baselineUpper = MAX_IBI_MS;

    if (ibi < baselineLower || ibi > baselineUpper) {
      return false;
    }
  }

  return true;
}


// ===============================
// 16. SQI 계산
// ===============================
float calculateSQI() {
  if (sqiCount < SQI_MIN_COUNT) {
    validWindowFlag = 0;
    return NAN;
  }

  float tempIbi[SQI_BUF_SIZE];
  float tempAmp[SQI_BUF_SIZE];

  for (int i = 0; i < sqiCount; i++) {
    int idx = (sqiIndex - sqiCount + i + SQI_BUF_SIZE) % SQI_BUF_SIZE;
    tempIbi[i] = sqiIbiBuf[idx];
    tempAmp[i] = sqiAmpBuf[idx];
  }

  float ibiCV = calcCV(tempIbi, sqiCount);
  float ampCV = calcCV(tempAmp, sqiCount);

  if (isnan(ibiCV) || isnan(ampCV)) {
    validWindowFlag = 0;
    return NAN;
  }

  float ibiScore = 100.0 * (1.0 - (ibiCV / MAX_IBI_CV));
  ibiScore = constrain(ibiScore, 0.0, 100.0);

  float ampScore = 100.0 * (1.0 - (ampCV / MAX_AMP_CV));
  ampScore = constrain(ampScore, 0.0, 100.0);

  float sqiScore = 0.6 * ibiScore + 0.4 * ampScore;

  validWindowFlag = (sqiScore >= MIN_SQI_SCORE) ? 1 : 0;

  return sqiScore;
}


// ===============================
// 17. HRV 지표 계산
// ===============================
void updateHrvMetrics() {
  if (ibiCount < 5) {
    meanHR = NAN;
    sdnn = NAN;
    rmssd = NAN;
    return;
  }

  float temp[IBI_BUF_SIZE];

  for (int i = 0; i < ibiCount; i++) {
    int idx = (ibiIndex - ibiCount + i + IBI_BUF_SIZE) % IBI_BUF_SIZE;
    temp[i] = ibiBuf[idx];
  }

  float meanIbi = calcMean(temp, ibiCount);
  meanHR = 60000.0 / meanIbi;

  sdnn = calcStd(temp, ibiCount);

  if (ibiCount < 2) {
    rmssd = NAN;
    return;
  }

  float sumDiffSq = 0;
  int diffCount = 0;

  for (int i = 1; i < ibiCount; i++) {
    float diff = temp[i] - temp[i - 1];
    sumDiffSq += diff * diff;
    diffCount++;
  }

  if (diffCount > 0) {
    rmssd = sqrt(sumDiffSq / diffCount);
  } else {
    rmssd = NAN;
  }
}


// ===============================
// 18. PPG z-score 및 최종 스트레스 점수 계산
// ===============================
String classifyStressLevel(float score) {
  if (isnan(score)) {
    return "";
  }

  if (score >= STRESS_THRESHOLD) {
    return "Stress";
  } else if (score >= NORMAL_THRESHOLD) {
    return "Mild";
  } else {
    return "Normal";
  }
}


void resetPpgStressScore() {
  meanHrZ = NAN;
  sdnnZ = NAN;
  rmssdZ = NAN;

  meanHrStressScore = NAN;
  sdnnStressScore = NAN;
  rmssdStressScore = NAN;

  ppgAvgScore = NAN;
  hrvScore = NAN;
  finalPpgStressScore = NAN;
  finalPpgStressLevel = "";
}


void updatePpgStressScore(String mode) {
  // REST / STRESS 구간에서만 z-score 기반 최종 점수 계산
  // TRANSITION은 과제 시작 직후 30초 과도기이므로 라벨링/분석에서 제외
  if (mode != "REST" && mode != "STRESS") {
    resetPpgStressScore();
    return;
  }

  // baseline이 충분하지 않으면 계산 보류
  if (
    baselineMetricCount < MIN_BASELINE_METRIC_COUNT_FOR_Z ||
    isnan(meanHR) ||
    isnan(sdnn) ||
    isnan(rmssd) ||
    isnan(baselineMeanHRMean) ||
    isnan(baselineMeanHRStd) ||
    isnan(baselineSDNNMean) ||
    isnan(baselineSDNNStd) ||
    isnan(baselineRMSSDMean) ||
    isnan(baselineRMSSDStd)
  ) {
    resetPpgStressScore();
    return;
  }

  // z-score 계산
  // baseline std가 너무 작으면 z-score가 폭주하므로 최소 표준편차를 적용
  float safeMeanHrStd = max(baselineMeanHRStd, MIN_MEAN_HR_STD_FOR_Z);
  float safeSdnnStd = max(baselineSDNNStd, MIN_SDNN_STD_FOR_Z);
  float safeRmssdStd = max(baselineRMSSDStd, MIN_RMSSD_STD_FOR_Z);

  // Mean HR: 증가 방향이 스트레스
  meanHrZ = (meanHR - baselineMeanHRMean) / safeMeanHrStd;

  // SDNN, RMSSD: 감소 방향이 스트레스
  sdnnZ = (sdnn - baselineSDNNMean) / safeSdnnStd;
  rmssdZ = (rmssd - baselineRMSSDMean) / safeRmssdStd;

  // 0~100 스트레스 점수 변환
  meanHrStressScore = clampFloat((meanHrZ / MAX_Z_FOR_SCORE) * 100.0, 0.0, 100.0);
  sdnnStressScore = clampFloat((-sdnnZ / MAX_Z_FOR_SCORE) * 100.0, 0.0, 100.0);
  rmssdStressScore = clampFloat((-rmssdZ / MAX_Z_FOR_SCORE) * 100.0, 0.0, 100.0);

  // 기본 평균 점수
  ppgAvgScore = (meanHrStressScore + sdnnStressScore + rmssdStressScore) / 3.0;

  // HRV 중심 점수
  hrvScore = (sdnnStressScore + rmssdStressScore) / 2.0;

  bool rmssdStress = rmssdStressScore >= STRESS_THRESHOLD;
  bool sdnnStress = sdnnStressScore >= STRESS_THRESHOLD;

  bool rmssdMild = rmssdStressScore >= NORMAL_THRESHOLD;
  bool sdnnMild = sdnnStressScore >= NORMAL_THRESHOLD;

  // 최종 PPG 점수 결정
  if (rmssdStress && sdnnStress) {
    finalPpgStressScore = max(ppgAvgScore, hrvScore);
    finalPpgStressLevel = "Stress";
  } else if (rmssdMild && sdnnMild) {
    finalPpgStressScore = max(ppgAvgScore, hrvScore);
    finalPpgStressLevel = classifyStressLevel(finalPpgStressScore);
  } else {
    finalPpgStressScore = ppgAvgScore;
    finalPpgStressLevel = classifyStressLevel(finalPpgStressScore);
  }
}


// ===============================
// 19. mode 반환
// ===============================
String getMode(unsigned long elapsedMs, bool noFinger) {
  if (noFinger) {
    return "NO_FINGER";
  }

  unsigned long t1 = STABILIZING_TIME_MS;
  unsigned long t2 = t1 + CALIBRATION_TIME_MS;
  unsigned long t3 = t2 + REST_TIME_MS;
  unsigned long t4 = t3 + TRANSITION_TIME_MS;
  unsigned long t5 = t4 + STRESS_TIME_MS;

  if (elapsedMs < t1) {
    return "STABILIZING";
  }

  if (elapsedMs < t2) {
    return "CALIBRATION";
  }

  if (elapsedMs < t3) {
    return "REST";
  }

  if (elapsedMs < t4) {
    return "TRANSITION";
  }

  if (elapsedMs < t5) {
    return "STRESS";
  }

  return "DONE";
}

// 성능 평가용 라벨링
// -1: 라벨링 제외(STABILIZING, CALIBRATION, TRANSITION, DONE, NO_FINGER)
//  0: 안정 상태 REST
//  1: 스트레스 상태 STRESS
int getClassLabel(String mode) {
  if (mode == "REST") return 0;
  if (mode == "STRESS") return 1;
  return -1;
}

int getLabelValid(String mode) {
  int label = getClassLabel(mode);
  return (label == 0 || label == 1) ? 1 : 0;
}




// ===============================
// 20. BLE 유틸 함수
// ===============================
uint8_t modeToCode(String mode) {
  if (mode == "STABILIZING") return 1;
  if (mode == "CALIBRATION") return 2;
  if (mode == "REST") return 3;
  if (mode == "TRANSITION") return 4;
  if (mode == "STRESS") return 5;
  if (mode == "DONE") return 6;
  if (mode == "NO_FINGER") return 7;
  return 0;
}

uint8_t levelToCode(String level) {
  if (level == "Normal") return 1;
  if (level == "Mild") return 2;
  if (level == "Stress") return 3;
  return 0;
}

void setupBle() {
  if (!BLE.begin()) {
    Serial.println("ERROR,BLE_START_FAILED");
    while (1);
  }

  BLE.setLocalName("BioGrip_Feature");
  BLE.setAdvertisedService(bioDataService);

  bioDataService.addCharacteristic(featureDataChar);
  bioDataService.addCharacteristic(controlPointChar);
  BLE.addService(bioDataService);

  memset(&latestBleFeatures, 0, sizeof(latestBleFeatures));
  featureDataChar.writeValue((uint8_t*)&latestBleFeatures, sizeof(latestBleFeatures));

  BLE.advertise();
  Serial.println("BLE_READY,BioGrip_Feature");
}

void updateBleFeatureData(String mode) {
  latestBleFeatures.mean_hr = isnan(meanHR) ? -1.0 : meanHR;
  latestBleFeatures.final_ppg_stress_score = isnan(finalPpgStressScore) ? -1.0 : finalPpgStressScore;
  latestBleFeatures.final_gsr_stress_score = isnan(gsr::latestFinalGsrStressScore) ? -1.0 : gsr::latestFinalGsrStressScore;
  latestBleFeatures.integrated_stress_score = isnan(latestIntegratedStressScore) ? -1.0 : latestIntegratedStressScore;
  latestBleFeatures.integrated_level = levelToCode(latestIntegratedStressLevel);
  latestBleFeatures.mode_code = modeToCode(mode);

  uint8_t flags = 0;
  if (latestPpgReadyForBle) flags |= 0x01;
  if (latestGsrReadyForBle) flags |= 0x02;
  if (getLabelValid(mode) == 1) flags |= 0x04;
  if (measurementActive) flags |= 0x08;
  latestBleFeatures.flags = flags;
}

void notifyBleFeatureData(String mode) {
  updateBleFeatureData(mode);
  featureDataChar.writeValue((uint8_t*)&latestBleFeatures, sizeof(latestBleFeatures));
}

void handleBleCommand() {
  if (!controlPointChar.written()) {
    return;
  }

  uint8_t command = controlPointChar.value();
  Serial.print("BLE_COMMAND,");
  Serial.println(command);

  // 1: 측정 시작 또는 재시작
  if (command == 1) {
    startMeasurement();
    measurementActive = true;
  }

  // 2: 측정 중지
  else if (command == 2) {
    measurementActive = false;
    Serial.println("MEASUREMENT_STOPPED_BY_BLE");
  }
}


// ===============================
// 20. 통합 CSV 출력 함수
// ===============================
void printCombinedCsvRow(unsigned long elapsedMs, String mode, long irRaw, float filteredPpg) {
  float integratedStressScore = NAN;
  String integratedStressLevel = "";

  // 통합 점수는 strict AND 방식이 아니라 조건부 평균 방식으로 계산한다.
  // PPG와 GSR이 모두 유효하면 두 값을 평균내고, 한쪽만 유효하면 해당 센서 값을 임시 통합값으로 사용한다.
  bool ppgReady =
    (mode == "REST" || mode == "STRESS") &&
    (validWindowFlag == 1) &&
    !isnan(finalPpgStressScore);

  bool gsrFresh = gsr::isLatestScoreFresh(millis());

  bool gsrReady =
    (gsr::latestMode == "REST" || gsr::latestMode == "STRESS") &&
    (gsr::latestContactOk == 1) &&
    (gsr::latestValidSampleFlag == 1) &&
    gsrFresh &&
    !isnan(gsr::latestFinalGsrStressScore);

  float integratedScoreSum = 0.0;
  int integratedScoreCount = 0;

  if (ppgReady) {
    integratedScoreSum += finalPpgStressScore;
    integratedScoreCount++;
  }

  if (gsrReady) {
    integratedScoreSum += gsr::latestFinalGsrStressScore;
    integratedScoreCount++;
  }

  if (integratedScoreCount > 0) {
    integratedStressScore = integratedScoreSum / integratedScoreCount;
    integratedStressLevel = classifyStressLevel(integratedStressScore);
  }

  latestIntegratedStressScore = integratedStressScore;
  latestIntegratedStressLevel = integratedStressLevel;
  latestPpgReadyForBle = ppgReady;
  latestGsrReadyForBle = gsrReady;

  Serial.print(elapsedMs);
  Serial.print(",");

  Serial.print(mode);
  Serial.print(",");

  Serial.print(irRaw);
  Serial.print(",");

  Serial.print(getClassLabel(mode));
  Serial.print(",");

  Serial.print(getLabelValid(mode));
  Serial.print(",");

  printFloatOrEmpty(filteredPpg, 4);
  Serial.print(",");

  printFloatOrEmpty(rawIbiMs, 2);
  Serial.print(",");

  printFloatOrEmpty(validIbiMs, 2);
  Serial.print(",");

  printFloatOrEmpty(meanHR, 2);
  Serial.print(",");

  printFloatOrEmpty(sdnn, 2);
  Serial.print(",");

  printFloatOrEmpty(rmssd, 2);
  Serial.print(",");

  printFloatOrEmpty(baselineMeanHRMean, 2);
  Serial.print(",");

  printFloatOrEmpty(baselineMeanHRStd, 2);
  Serial.print(",");

  printFloatOrEmpty(baselineSDNNMean, 2);
  Serial.print(",");

  printFloatOrEmpty(baselineSDNNStd, 2);
  Serial.print(",");

  printFloatOrEmpty(baselineRMSSDMean, 2);
  Serial.print(",");

  printFloatOrEmpty(baselineRMSSDStd, 2);
  Serial.print(",");

  printFloatOrEmpty(baselineIbiMean, 2);
  Serial.print(",");

  printFloatOrEmpty(baselineIbiStd, 2);
  Serial.print(",");

  printFloatOrEmpty(currentSQI, 2);
  Serial.print(",");

  Serial.print(validWindowFlag);
  Serial.print(",");

  printFloatOrEmpty(meanHrZ, 4);
  Serial.print(",");

  printFloatOrEmpty(sdnnZ, 4);
  Serial.print(",");

  printFloatOrEmpty(rmssdZ, 4);
  Serial.print(",");

  printFloatOrEmpty(meanHrStressScore, 2);
  Serial.print(",");

  printFloatOrEmpty(sdnnStressScore, 2);
  Serial.print(",");

  printFloatOrEmpty(rmssdStressScore, 2);
  Serial.print(",");

  printFloatOrEmpty(ppgAvgScore, 2);
  Serial.print(",");

  printFloatOrEmpty(hrvScore, 2);
  Serial.print(",");

  printFloatOrEmpty(finalPpgStressScore, 2);
  Serial.print(",");

  Serial.print(finalPpgStressLevel);
  Serial.print(",");

  Serial.print(ibiCount);
  Serial.print(",");

  Serial.print(baselineMetricCount);
  Serial.print(",");

  Serial.print(noFingerCount);
  Serial.print(",");

  Serial.print(gsr::latestMode);
  Serial.print(",");

  Serial.print(gsr::latestSampleIndex);
  Serial.print(",");

  Serial.print(gsr::latestRawGsr);
  Serial.print(",");

  printFloatOrEmpty(gsr::latestMedianGsr, 2);
  Serial.print(",");

  printFloatOrEmpty(gsr::latestSclBaseline, 2);
  Serial.print(",");

  printFloatOrEmpty(gsr::latestScrDetectBaseline, 2);
  Serial.print(",");

  printFloatOrEmpty(gsr::latestPhasic, 2);
  Serial.print(",");

  printFloatOrEmpty(gsr::latestPhasicPos, 2);
  Serial.print(",");

  printFloatOrEmpty(gsr::scrThreshold, 2);
  Serial.print(",");

  printFloatOrEmpty(gsr::MIN_SCR_THRESHOLD_ADC, 2);
  Serial.print(",");

  printFloatOrEmpty(gsr::MIN_SCR_RISE_ADC, 2);
  Serial.print(",");

  printFloatOrEmpty(gsr::latestSclMean30s, 2);
  Serial.print(",");

  printFloatOrEmpty(gsr::latestScrAmplitudeMean30s, 2);
  Serial.print(",");

  printFloatOrEmpty(gsr::latestNsScrFrequencyPerMin, 2);
  Serial.print(",");

  printFloatOrEmpty(gsr::latestSclMeanZ, 4);
  Serial.print(",");

  printFloatOrEmpty(gsr::latestScrAmplitudeMeanZ, 4);
  Serial.print(",");

  printFloatOrEmpty(gsr::latestNsScrFrequencyZ, 4);
  Serial.print(",");

  printFloatOrEmpty(gsr::latestSclStressScore, 2);
  Serial.print(",");

  printFloatOrEmpty(gsr::latestScrAmplitudeStressScore, 2);
  Serial.print(",");

  printFloatOrEmpty(gsr::latestNsScrFrequencyStressScore, 2);
  Serial.print(",");

  printFloatOrEmpty(gsr::latestFinalGsrStressScore, 2);
  Serial.print(",");

  Serial.print(gsr::latestFeatureReady);
  Serial.print(",");

  Serial.print(gsr::latestContactOk);
  Serial.print(",");

  Serial.print(gsr::latestValidSampleFlag);
  Serial.print(",");

  printFloatOrEmpty(integratedStressScore, 2);
  Serial.print(",");

  Serial.println(integratedStressLevel);
}

// ===============================
// 21. PPG 전체 변수 초기화 함수
// ===============================
void resetPpgVariables() {
  // 필터 상태
  hp_x1 = 0; hp_x2 = 0; hp_y1 = 0; hp_y2 = 0;
  lp_x1 = 0; lp_x2 = 0; lp_y1 = 0; lp_y2 = 0;

  // Peak 검출
  ppg_prev2 = 0; ppg_prev1 = 0; ppg_curr = 0;
  abs_ema = 0;
  lastPeakTime = 0;
  rawIbiMs = NAN;
  validIbiMs = NAN;

  // Median buffer
  for (int i = 0; i < MEDIAN_BUF_SIZE; i++) medianBuf[i] = 0;
  medianCount = 0;
  medianIndex = 0;

  // IBI buffer
  for (int i = 0; i < IBI_BUF_SIZE; i++) ibiBuf[i] = 0;
  ibiCount = 0;
  ibiIndex = 0;
  meanHR = NAN; sdnn = NAN; rmssd = NAN;

  // Baseline HRV
  baselineMetricCount = 0;
  baselineMeanHRMean = NAN; baselineMeanHRStd = NAN;
  baselineSDNNMean = NAN;   baselineSDNNStd = NAN;
  baselineRMSSDMean = NAN;  baselineRMSSDStd = NAN;
  meanHR_m = 0; meanHR_s = 0;
  sdnn_m = 0;   sdnn_s = 0;
  rmssd_m = 0;  rmssd_s = 0;

  // Baseline IBI
  baselineIbiCount = 0;
  baselineIbiMean = NAN; baselineIbiStd = NAN;
  baselineIbi_m = 0;     baselineIbi_s = 0;

  // SQI
  for (int i = 0; i < SQI_BUF_SIZE; i++) { sqiIbiBuf[i] = 0; sqiAmpBuf[i] = 0; }
  sqiCount = 0; sqiIndex = 0;
  currentSQI = NAN; validWindowFlag = 0;

  // Z-score / stress score
  meanHrZ = NAN; sdnnZ = NAN; rmssdZ = NAN;
  meanHrStressScore = NAN; sdnnStressScore = NAN; rmssdStressScore = NAN;
  ppgAvgScore = NAN; hrvScore = NAN; finalPpgStressScore = NAN;
  finalPpgStressLevel = "";

  // 접촉 카운터
  consecutiveNoFinger = 0;
  noFingerCount = 0;
}


// ===============================
// 22. 전체 시스템 초기화 함수 (재측정 시작)
// ===============================
void startMeasurement() {
  measurementActive = true;
  resetPpgVariables();
  gsr::resetAll();

  unsigned long t0 = millis();
  startTime           = t0;
  lastSampleTime      = t0;
  lastPrintTime       = t0;
  gsr::lastSampleTime = t0;
  measurementDonePrinted = false;
  latestIntegratedStressScore = NAN;
  latestIntegratedStressLevel = "";
  latestPpgReadyForBle = false;
  latestGsrReadyForBle = false;
  memset(&latestBleFeatures, 0, sizeof(latestBleFeatures));

  Serial.println(
    "time_ms,ppg_mode,ir_raw,class_label,label_valid,filtered_ppg,ibi_ms,valid_ibi_ms,"
    "mean_hr,sdnn,rmssd,"
    "baseline_mean_hr_mean,baseline_mean_hr_std,"
    "baseline_sdnn_mean,baseline_sdnn_std,"
    "baseline_rmssd_mean,baseline_rmssd_std,"
    "baseline_ibi_mean,baseline_ibi_std,"
    "sqi_score,valid_window_flag,"
    "mean_hr_z,sdnn_z,rmssd_z,"
    "mean_hr_stress_score,sdnn_stress_score,rmssd_stress_score,"
    "ppg_avg_score,hrv_score,final_ppg_stress_score,final_ppg_stress_level,"
    "ibi_count,baseline_count,no_finger_count,"
    "gsr_mode,gsr_sample_index,raw_gsr,median_gsr,scl_baseline,scr_detect_baseline,"
    "phasic,phasic_pos,gsr_scr_threshold,gsr_min_scr_threshold_adc,gsr_min_scr_rise_adc,scl_mean_30s,scr_amplitude_mean_30s,ns_scr_frequency_per_min,"
    "scl_mean_z,scr_amplitude_mean_z,ns_scr_frequency_z,"
    "scl_stress_score,scr_amplitude_stress_score,ns_scr_frequency_stress_score,"
    "final_gsr_stress_score,gsr_feature_ready,gsr_contact_ok,gsr_valid_sample_flag,"
    "integrated_stress_score,integrated_stress_level"
  );
}


// ===============================
// 23. setup
// ===============================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR,MAX30102_NOT_FOUND");
    while (1);
  }

  byte ledBrightness = 0x5F;
  byte sampleAverage = 4;
  byte ledMode = 2;
  int sampleRate = 100;
  int pulseWidth = 411;
  int adcRange = 16384;

  particleSensor.setup(
    ledBrightness,
    sampleAverage,
    ledMode,
    sampleRate,
    pulseWidth,
    adcRange
  );

  particleSensor.setPulseAmplitudeRed(0x5F);
  particleSensor.setPulseAmplitudeIR(0x5F);

  gsr::setup();
  setupBle();

  // PC에서는 기존처럼 'S' 명령을 보내면 시작되고,
  // BLE에서는 Control Point에 1을 쓰면 시작된다.
  Serial.println("READY");
}


// ===============================
// 24. loop
// ===============================
void loop() {
  BLE.poll();
  handleBleCommand();

  // PC 시리얼 명령 처리
  // S/s: 측정 시작, R/r: 재측정, X/x: 측정 중지
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'S' || cmd == 's' || cmd == 'R' || cmd == 'r') {
      Serial.println("START");
      delay(100);
      startMeasurement();
      return;
    } else if (cmd == 'X' || cmd == 'x') {
      measurementActive = false;
      Serial.println("MEASUREMENT_STOPPED_BY_SERIAL");
      return;
    }
  }

  if (!measurementActive) {
    return;
  }

  gsr::loop();

  unsigned long now = millis();

  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) {
    return;
  }

  lastSampleTime = now;

  unsigned long elapsedMs = now - startTime;

  long irRaw = particleSensor.getIR();

  // 연속 미접촉 판정
  if (irRaw <= 0 || irRaw < NO_FINGER_IR_THRESHOLD) {
    consecutiveNoFinger++;
  } else {
    consecutiveNoFinger = 0;
  }

  bool noFinger = consecutiveNoFinger >= NO_FINGER_CONFIRM_COUNT;

  if (noFinger) {
    noFingerCount++;
  }

  String mode = getMode(elapsedMs, noFinger);

  // 11분 15초 측정 완료
  // 무한 대기로 막지 않고, 완료 메시지만 한 번 출력한 뒤 loop를 계속 돌린다.
  // 그래야 DONE 이후에도 PC에서 R/r 명령을 보내 재측정을 시작할 수 있다.
  if (mode == "DONE") {
    if (!measurementDonePrinted) {
      Serial.println("MEASUREMENT_DONE");
      measurementDonePrinted = true;
      measurementActive = false;
      notifyBleFeatureData(mode);
    }
    return;
  }

  float filteredPpg = NAN;

  // 손가락이 있다고 판단되는 경우에만 필터, peak, IBI 처리
  if (!noFinger && irRaw > 0) {
    filteredPpg = applyBandpass((float)irRaw);

    if (mode != "STABILIZING") {
      ppg_prev2 = ppg_prev1;
      ppg_prev1 = ppg_curr;
      ppg_curr = filteredPpg;

      abs_ema = (1.0 - ABS_EMA_ALPHA) * abs_ema + ABS_EMA_ALPHA * abs(filteredPpg);
      float peakThreshold = abs_ema * PEAK_THRESHOLD_RATIO;

      // local peak는 ppg_prev1, 즉 한 샘플 전 지점에서 발생한 것으로 판단한다.
      // 따라서 IBI 계산과 refractory 판단에도 현재 now가 아니라 한 샘플 전 시간을 사용한다.
      unsigned long peakTimeMs =
        (now >= SAMPLE_INTERVAL_MS) ? (now - SAMPLE_INTERVAL_MS) : now;

      bool isPeak = false;

      if (
        ppg_prev1 > ppg_prev2 &&
        ppg_prev1 > ppg_curr &&
        ppg_prev1 > peakThreshold &&
        peakTimeMs - lastPeakTime > REFRACTORY_MS
      ) {
        isPeak = true;
      }

      if (isPeak) {
        if (lastPeakTime > 0) {
          rawIbiMs = (float)(peakTimeMs - lastPeakTime);
          float peakAmp = ppg_prev1;

          // 1. basic IBI 검사
          if (isBasicValidIbi(rawIbiMs)) {

            // 2. local median은 basic 통과 후 업데이트
            addMedianBuffer(rawIbiMs);

            // 3. CALIBRATION 구간 baseline IBI 누적
            if (mode == "CALIBRATION") {
              updateBaselineIbi(rawIbiMs);
            }

            // 4. baseline IBI 범위 검사
            if (isBaselineRangeValidIbi(rawIbiMs)) {

              // 5. SQI window 추가
              addSqiBuffer(rawIbiMs, peakAmp);

              // 6. SQI 계산
              currentSQI = calculateSQI();

              // 7. SQI 통과 시 최종 valid IBI 인정
              if (validWindowFlag == 1) {
                validIbiMs = rawIbiMs;

                addIbiBuffer(validIbiMs);
                updateHrvMetrics();

                // CALIBRATION 구간에서는 최종 valid IBI 기반 HRV baseline 누적
                if (mode == "CALIBRATION") {
                  updateBaselineMetrics();
                }
              } else {
                validIbiMs = NAN;
              }

            } else {
              validIbiMs = NAN;
            }

          } else {
            validIbiMs = NAN;
          }
        }

        lastPeakTime = peakTimeMs;
      }
    }
  } else {
    filteredPpg = NAN;
    validIbiMs = NAN;
  }


  // 1초마다 CSV 형식 출력
  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;

    // 출력 직전 현재 HRV와 baseline 기준으로 PPG 스트레스 점수 갱신
    updatePpgStressScore(mode);

    // PPG + GSR + 통합 스트레스 점수를 한 줄의 CSV로 출력
    printCombinedCsvRow(elapsedMs, mode, irRaw, filteredPpg);

    // 같은 시점의 핵심 결과를 BLE Notify로 전송
    notifyBleFeatureData(mode);

    // 출력 후 다음 1초 구간의 새 IBI만 표시하기 위해 리셋
    rawIbiMs = NAN;
    validIbiMs = NAN;
  }
}