#include <Wire.h>
#include "MAX30105.h"
#include <ArduinoBLE.h>

// ==========================================
// 1. 공통 설정 및 시스템 변수
// ==========================================
const unsigned long COMMON_STABILIZING_TIME_MS = 15000UL;
const unsigned long COMMON_CALIBRATION_TIME_MS = 60000UL;
const unsigned long COMMON_REST_TIME_MS        = 300000UL;
const unsigned long COMMON_TRANSITION_TIME_MS  = 30000UL;
const unsigned long COMMON_STRESS_TIME_MS      = 270000UL;
const unsigned long COMMON_BASELINE_END_MS     = COMMON_STABILIZING_TIME_MS + COMMON_CALIBRATION_TIME_MS;
const unsigned long COMMON_FEATURE_WINDOW_MS   = 30000UL;

const bool ENABLE_SERIAL_CSV = true;  
bool sys_csvHeaderPrinted = false;

bool  sys_measurementActive = false;
float sys_integratedStressZ = NAN;
String sys_integratedStressState = "";
int   sys_validFeatureCount = 0;
bool  sys_ppgReadyForBle = false;
bool  sys_gsrReadyForBle = false;

// ==========================================
// 2. BLE 설정
// ==========================================
BLEService bioDataService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic featureDataChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 32);
BLEUnsignedCharCharacteristic controlPointChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLEWrite);

#pragma pack(1)
struct BleFeatureData {
  uint16_t seq;
  float mean_hr;
  float final_ppg_stress_z;
  float final_gsr_stress_z;
  float integrated_stress_z;
  uint8_t integrated_level;
  uint8_t mode_code;
  uint8_t flags;
};
#pragma pack()

BleFeatureData ble_latestFeatures;
uint16_t ble_seq = 0;
unsigned long ble_lastNotifyTime = 0;
const unsigned long BLE_NOTIFY_INTERVAL_MS = 50000UL; 

// ==========================================
// 3. 공통 유틸리티 및 수학 계산
// ==========================================
void printCsvFloat(float x, int digits, bool isLast = false) {
  if (isnan(x)) Serial.print("");
  else Serial.print(x, digits);
  if (!isLast) Serial.print(",");
}

void printCsvString(String s, bool isLast = false) {
  Serial.print(s);
  if (!isLast) Serial.print(",");
}

String classifyBinaryState(float stressZ) {
  if (isnan(stressZ)) return "";
  if (stressZ >= 1.0) return "Stress";
  return "Rest";
}

String getMode(unsigned long elapsedMs, bool noFinger) {
  if (noFinger) return "NO_FINGER";
  if (elapsedMs < COMMON_STABILIZING_TIME_MS) return "STABILIZING";
  if (elapsedMs < COMMON_BASELINE_END_MS) return "CALIBRATION";
  if (elapsedMs < COMMON_BASELINE_END_MS + COMMON_REST_TIME_MS) return "REST";
  if (elapsedMs < COMMON_BASELINE_END_MS + COMMON_REST_TIME_MS + COMMON_TRANSITION_TIME_MS) return "TRANSITION";
  if (elapsedMs < COMMON_BASELINE_END_MS + COMMON_REST_TIME_MS + COMMON_TRANSITION_TIME_MS + COMMON_STRESS_TIME_MS) return "STRESS";
  return "DONE";
}

uint8_t getModeCode(String mode) {
  if (mode == "STABILIZING") return 0;
  if (mode == "CALIBRATION") return 1;
  if (mode == "REST") return 2;
  if (mode == "TRANSITION") return 3;
  if (mode == "STRESS") return 4;
  return 5;
}

void sys_updateWelford(float x, int &countVal, float &meanVal, float &m2Val) {
  countVal++;
  float delta = x - meanVal;
  meanVal += delta / countVal;
  float delta2 = x - meanVal;
  m2Val += delta * delta2;
}

float sys_welfordSampleStd(float m2Val, int countVal) {
  if (countVal <= 1) return 0.0;
  float variance = m2Val / (countVal - 1);
  return (variance > 0.0) ? sqrt(variance) : 0.0;
}

float sys_computeZScore(float value, float meanVal, float stdVal, float minStd) {
  float s = (stdVal < minStd) ? minStd : stdVal;
  return (value - meanVal) / s;
}

float sys_calcMean(float arr[], int n) { 
  if(n <= 0) return NAN;
  float s = 0; for(int i=0; i<n; i++) s += arr[i]; 
  return s/n; 
}

float sys_calcStd(float arr[], int n) { 
  if(n < 2) return NAN; 
  float m = sys_calcMean(arr, n), sq = 0; 
  for(int i=0; i<n; i++) sq += (arr[i] - m) * (arr[i] - m); 
  return sqrt(sq/(n-1)); 
}

float sys_calcCV(float arr[], int n) { 
  float m = sys_calcMean(arr, n); 
  if(isnan(m) || m == 0) return NAN; 
  return sys_calcStd(arr, n) / m; 
}

// ==========================================
// 4. GSR 전용 변수
// ==========================================
#define GSR_PIN A0
const float GSR_ADC_MAX_VALUE = 1023.0; 
const unsigned long GSR_SAMPLE_INTERVAL_MS = 100;
unsigned long gsr_lastSampleTime = 0;
unsigned long gsr_startTimeMs = 0;
int gsr_sampleIndex = 0;
int gsr_latestRawAdc = 0;

const int gsr_MIN_VALID_GSR_ADC = 10;
const int gsr_CONTACT_RECOVERY_SAMPLES = 10;
int gsr_contactRecoveryCount = 0;
int gsr_contactOk = 0;
int gsr_validSampleFlag = 0;

const int gsr_MEDIAN_N = 3; 
float gsr_medBuf[3] = {0,0,0}; 
int gsr_medIndex = 0; 
bool gsr_medFilled = false;

const int gsr_MA_N = 150; 
float gsr_maBuf[150]; 
int gsr_maIndex = 0, gsr_maCount = 0; 
float gsr_maSum = 0.0;

const float gsr_SCR_BASELINE_ALPHA = 0.02;
float gsr_scrDetectBaseline = 0.0; 
bool gsr_scrBaselineInitialized = false;
float gsr_phasicPrev2 = 0.0, gsr_phasicPrev1 = 0.0, gsr_phasicCurr = 0.0;

const float gsr_THRESHOLD_K = 2.5;
const float gsr_MIN_SCR_THRESHOLD_ADC = 6.0;
const float gsr_MAX_SCR_THRESHOLD_ADC = 15.0;
const float gsr_MIN_SCR_RISE_ADC = 2.0; // 최소 상승폭
const int gsr_REFRACTORY_SAMPLES = 10;
int gsr_lastScrSampleIndex = -9999;
float gsr_scrThreshold = 6.0;

int gsr_baselineCount = 0; float gsr_baselinePhasicMean = 0.0, gsr_baselinePhasicM2 = 0.0, gsr_baselinePhasicStd = 0.0;
int gsr_baselineSclCount = 0; float gsr_baselineSclMean = 0.0, gsr_baselineSclM2 = 0.0, gsr_baselineSclStd = 0.0;
int gsr_baselinePhasicPosCount = 0; float gsr_baselinePhasicPosMean = 0.0, gsr_baselinePhasicPosM2 = 0.0, gsr_baselinePhasicPosStd = 0.0;
bool gsr_baselineReady = false;

int gsr_calibScrCount = 0; 
float gsr_baselineNsScrFreqMean = 0.0; 
float gsr_baselineNsScrFreqStd = 2.0; 

float gsr_windowSclSum = 0.0; int gsr_windowSampleCount = 0;
float gsr_windowScrAmpSum = 0.0; int gsr_windowScrCount = 0;
String gsr_activeFeatureWindowMode = ""; 
String gsr_previousMode = ""; 
String gsr_latestMode = "STABILIZING";

float gsr_latestSclMean30s = NAN; float gsr_latestScrAmplitudeMean30s = NAN; float gsr_latestNsScrFrequencyPerMin = NAN;
float gsr_sclStressZ = NAN; float gsr_scrAmplitudeStressZ = NAN; float gsr_nsScrFrequencyStressZ = NAN;
float gsr_finalStressZ = NAN; 
int gsr_featureReady = 0; unsigned long gsr_latestScoreUpdateMs = 0;

// ==========================================
// 5. PPG 전용 변수
// ==========================================
MAX30105 ppg_sensor;
const unsigned long PPG_SAMPLE_INTERVAL_MS = 10;
unsigned long ppg_lastSampleTime = 0;
unsigned long ppg_startTimeMs = 0;
long ppg_latestIrRaw = 0; 
float ppg_latestFiltered = 0; 

const long ppg_NO_FINGER_IR_THRESHOLD = 10000;
const int ppg_NO_FINGER_CONFIRM_COUNT = 10;
int ppg_consecutiveNoFinger = 0;

float ppg_baselineEma = 0.0; bool ppg_baselineInitialized = false;
const float ppg_BASELINE_EMA_ALPHA = 0.01;

float ppg_prev2 = 0.0, ppg_prev1 = 0.0, ppg_curr = 0.0, ppg_absEma = 0.0;
const float ppg_ABS_EMA_ALPHA = 0.01; const float ppg_PEAK_THRESHOLD_RATIO = 0.35;
const unsigned long ppg_REFRACTORY_MS = 300; unsigned long ppg_lastPeakTime = 0;

float ppg_rawIbiMs = NAN, ppg_validIbiMs = NAN;
const float ppg_MIN_IBI_MS = 300.0, ppg_MAX_IBI_MS = 1500.0;

int ppg_baselineHRCount = 0; float ppg_meanHR_m = 0, ppg_meanHR_s = 0;
int ppg_baselineSDNNCount = 0; float ppg_sdnn_m = 0, ppg_sdnn_s = 0;
int ppg_baselineRMSSDCount = 0; float ppg_rmssd_m = 0, ppg_rmssd_s = 0;

float ppg_baselineMeanHRMean = NAN, ppg_baselineMeanHRStd = NAN;
float ppg_baselineSDNNMean = NAN, ppg_baselineSDNNStd = NAN;
float ppg_baselineRMSSDMean = NAN, ppg_baselineRMSSDStd = NAN;

float ppg_meanHR = NAN, ppg_sdnn = NAN, ppg_rmssd = NAN;
int ppg_validWindowFlag = 0;

float ppg_sdnnStressZ = NAN, ppg_rmssdStressZ = NAN;
float ppg_finalStressZ = NAN;

const int ppg_IBI_BUF_SIZE = 30;
float ppg_ibiBuf[30]; int ppg_ibiCount = 0, ppg_ibiIndex = 0;
const int ppg_SQI_BUF_SIZE = 10;
float ppg_sqiIbiBuf[10], ppg_sqiAmpBuf[10]; int ppg_sqiCount = 0, ppg_sqiIndex = 0;


// ==========================================
// 6. 초기화 함수
// ==========================================
void sys_resetAll() {
  gsr_medIndex = 0; gsr_medFilled = false; gsr_maIndex = 0; gsr_maCount = 0; gsr_maSum = 0;
  gsr_scrDetectBaseline = 0; gsr_scrBaselineInitialized = false;
  gsr_baselineReady = false; gsr_scrThreshold = 6.0; gsr_calibScrCount = 0;
  gsr_baselineCount = 0; gsr_baselineSclCount = 0; gsr_baselinePhasicPosCount = 0;
  gsr_featureReady = 0; gsr_finalStressZ = NAN;
  
  // 상태 변수 초기화
  gsr_activeFeatureWindowMode = "";
  gsr_previousMode = "";
  gsr_latestMode = "STABILIZING";
  
  ppg_baselineEma = 0; ppg_baselineInitialized = false; ppg_lastPeakTime = 0;
  ppg_baselineHRCount = 0; ppg_baselineSDNNCount = 0; ppg_baselineRMSSDCount = 0;
  ppg_ibiCount = 0; ppg_sqiCount = 0; ppg_validWindowFlag = 0;
  ppg_finalStressZ = NAN; sys_integratedStressZ = NAN; sys_integratedStressState = "";
  sys_validFeatureCount = 0;
}

// ==========================================
// 7. GSR 프로세스
// ==========================================
void gsr_processSample(unsigned long elapsedMs, String mode) {
  gsr_latestMode = mode;

  // [수정 1] GSR Mode Change 로직 추가 (구간 전환 시 Window 초기화 및 모드 설정)
  bool isFeatureMode = (mode == "REST" || mode == "TRANSITION" || mode == "STRESS");
  if (mode != gsr_previousMode) {
    gsr_windowSclSum = 0; 
    gsr_windowSampleCount = 0; 
    gsr_windowScrAmpSum = 0; 
    gsr_windowScrCount = 0;
    gsr_phasicPrev2 = 0; 
    gsr_phasicPrev1 = 0; 
    gsr_phasicCurr = 0; 
    gsr_lastScrSampleIndex = -9999;
    gsr_finalStressZ = NAN; 
    gsr_featureReady = 0; 
    gsr_latestScoreUpdateMs = 0;
    
    // 측정 구간인 경우에만 activeFeatureWindowMode 활성화
    gsr_activeFeatureWindowMode = isFeatureMode ? mode : "";
    gsr_previousMode = mode;
  }

  gsr_latestRawAdc = analogRead(GSR_PIN);
  float analysisGsr = GSR_ADC_MAX_VALUE - (float)gsr_latestRawAdc;
  
  gsr_contactOk = (gsr_latestRawAdc >= gsr_MIN_VALID_GSR_ADC) ? 1 : 0;
  if (!gsr_contactOk) { 
    gsr_contactRecoveryCount = gsr_CONTACT_RECOVERY_SAMPLES; 
    gsr_validSampleFlag = 0; 
    gsr_sampleIndex++; 
    return; 
  }

  // 미디언 필터 N=3
  gsr_medBuf[gsr_medIndex++] = analysisGsr; 
  if (gsr_medIndex >= gsr_MEDIAN_N) { gsr_medIndex = 0; gsr_medFilled = true; }
  
  float medianGsr = analysisGsr;
  if (gsr_medFilled) {
    float a = gsr_medBuf[0], b = gsr_medBuf[1], c = gsr_medBuf[2];
    if ((a >= b && a <= c) || (a <= b && a >= c)) medianGsr = a;
    else if ((b >= a && b <= c) || (b <= a && b >= c)) medianGsr = b;
    else medianGsr = c;
  }

  // SCL MA Filter (N=150)
  if (gsr_maCount < gsr_MA_N) { 
    gsr_maBuf[gsr_maIndex] = medianGsr; 
    gsr_maSum += medianGsr; 
    gsr_maCount++; 
  } else { 
    gsr_maSum = gsr_maSum - gsr_maBuf[gsr_maIndex] + medianGsr; 
    gsr_maBuf[gsr_maIndex] = medianGsr; 
  }
  gsr_maIndex = (gsr_maIndex + 1) % gsr_MA_N; 
  float sclBaseline = gsr_maSum / gsr_maCount;

  // SCR EMA Freeze 로직
  if (!gsr_scrBaselineInitialized) { 
    gsr_scrDetectBaseline = medianGsr; 
    gsr_scrBaselineInitialized = true; 
  } else {
    if (gsr_baselineReady && (medianGsr - gsr_scrDetectBaseline > gsr_scrThreshold * 0.7)) {
      // Hold EMA
    } else {
      gsr_scrDetectBaseline = gsr_SCR_BASELINE_ALPHA * medianGsr + (1.0 - gsr_SCR_BASELINE_ALPHA) * gsr_scrDetectBaseline;
    }
  }
  
  gsr_validSampleFlag = (gsr_contactRecoveryCount > 0) ? (gsr_contactRecoveryCount--, 0) : 1;
  float phasic = medianGsr - gsr_scrDetectBaseline;
  float phasicPos = (phasic < 0.0) ? 0.0 : phasic;

  // Calibration Finalize
  if (!gsr_baselineReady && elapsedMs >= COMMON_BASELINE_END_MS) {
    if (gsr_baselineCount > 0) {
      gsr_baselinePhasicStd = sys_welfordSampleStd(gsr_baselinePhasicM2, gsr_baselineCount);
      gsr_baselineSclStd = sys_welfordSampleStd(gsr_baselineSclM2, gsr_baselineSclCount);
      gsr_baselinePhasicPosStd = sys_welfordSampleStd(gsr_baselinePhasicPosM2, gsr_baselinePhasicPosCount);
      gsr_scrThreshold = constrain(gsr_THRESHOLD_K * gsr_baselinePhasicStd, gsr_MIN_SCR_THRESHOLD_ADC, gsr_MAX_SCR_THRESHOLD_ADC);
      
      gsr_baselineNsScrFreqMean = gsr_calibScrCount * (60000.0 / COMMON_CALIBRATION_TIME_MS); 
      gsr_baselineNsScrFreqStd = 2.0; 
      gsr_baselineReady = true;
    }
  }

  // Baseline Update
  if (mode == "CALIBRATION" && gsr_validSampleFlag == 1) {
    sys_updateWelford(phasic, gsr_baselineCount, gsr_baselinePhasicMean, gsr_baselinePhasicM2);
    sys_updateWelford(sclBaseline, gsr_baselineSclCount, gsr_baselineSclMean, gsr_baselineSclM2);
    sys_updateWelford(phasicPos, gsr_baselinePhasicPosCount, gsr_baselinePhasicPosMean, gsr_baselinePhasicPosM2);
  }

  // Peak Detection
  if ((isFeatureMode || mode == "CALIBRATION") && gsr_validSampleFlag == 1) {
    gsr_phasicPrev2 = gsr_phasicPrev1; gsr_phasicPrev1 = gsr_phasicCurr; gsr_phasicCurr = phasicPos;
    
    bool localPeak = (gsr_phasicPrev1 > gsr_phasicPrev2) && (gsr_phasicPrev1 >= gsr_phasicCurr);
    bool aboveThresh = (gsr_phasicPrev1 >= gsr_scrThreshold);
    // [수정 2] GSR 최소 상승폭 조건 (MIN_SCR_RISE_ADC) 적용
    bool riseOk = ((gsr_phasicPrev1 - gsr_phasicPrev2) >= gsr_MIN_SCR_RISE_ADC);

    bool isPeak = localPeak && aboveThresh && riseOk;

    if (isPeak && ((gsr_sampleIndex - 1) - gsr_lastScrSampleIndex) >= gsr_REFRACTORY_SAMPLES) {
      gsr_lastScrSampleIndex = gsr_sampleIndex - 1;
      if (mode == "CALIBRATION") gsr_calibScrCount++; 
      else if (gsr_activeFeatureWindowMode == mode && gsr_baselineReady) {
        gsr_windowScrCount++; gsr_windowScrAmpSum += gsr_phasicPrev1;
      }
    }
  }

  // Feature Extraction
  if (isFeatureMode && gsr_activeFeatureWindowMode == mode && gsr_baselineReady && gsr_validSampleFlag == 1) {
    gsr_windowSclSum += sclBaseline; gsr_windowSampleCount++;
    if (gsr_windowSampleCount >= 300) {
      gsr_latestSclMean30s = gsr_windowSclSum / gsr_windowSampleCount;
      gsr_latestScrAmplitudeMean30s = (gsr_windowScrCount > 0) ? (gsr_windowScrAmpSum / gsr_windowScrCount) : 0.0;
      gsr_latestNsScrFrequencyPerMin = gsr_windowScrCount * 2.0;

      gsr_sclStressZ = sys_computeZScore(gsr_latestSclMean30s, gsr_baselineSclMean, gsr_baselineSclStd, 1.0);
      gsr_scrAmplitudeStressZ = sys_computeZScore(gsr_latestScrAmplitudeMean30s, gsr_baselinePhasicPosMean, gsr_baselinePhasicPosStd, 1.0);
      gsr_nsScrFrequencyStressZ = sys_computeZScore(gsr_latestNsScrFrequencyPerMin, gsr_baselineNsScrFreqMean, gsr_baselineNsScrFreqStd, 2.0);

      float zSum = 0; int zCount = 0;
      if (!isnan(gsr_sclStressZ)) { zSum += gsr_sclStressZ; zCount++; }
      if (gsr_windowScrCount > 0 && !isnan(gsr_scrAmplitudeStressZ)) { zSum += gsr_scrAmplitudeStressZ; zCount++; }
      if (gsr_windowScrCount > 0 && !isnan(gsr_nsScrFrequencyStressZ)) { zSum += gsr_nsScrFrequencyStressZ; zCount++; }
      gsr_finalStressZ = (zCount > 0) ? (zSum / zCount) : NAN;

      gsr_featureReady = 1; gsr_latestScoreUpdateMs = millis();
      gsr_windowSclSum = 0; gsr_windowSampleCount = 0; gsr_windowScrAmpSum = 0; gsr_windowScrCount = 0;
    }
  }
  gsr_sampleIndex++;
}

// ==========================================
// 8. PPG 프로세스
// ==========================================
void ppg_processSample(unsigned long now, unsigned long elapsedMs, String mode) {
  ppg_latestIrRaw = ppg_sensor.getIR();
  
  if (ppg_latestIrRaw < ppg_NO_FINGER_IR_THRESHOLD) ppg_consecutiveNoFinger++;
  else ppg_consecutiveNoFinger = 0;
  if (ppg_consecutiveNoFinger >= ppg_NO_FINGER_CONFIRM_COUNT) return;

  if (!ppg_baselineInitialized) { ppg_baselineEma = ppg_latestIrRaw; ppg_baselineInitialized = true; }
  else ppg_baselineEma = ppg_BASELINE_EMA_ALPHA * ppg_latestIrRaw + (1.0 - ppg_BASELINE_EMA_ALPHA) * ppg_baselineEma;
  
  ppg_latestFiltered = ppg_latestIrRaw - ppg_baselineEma;

  if (mode == "STABILIZING") return;

  ppg_prev2 = ppg_prev1; ppg_prev1 = ppg_curr; ppg_curr = ppg_latestFiltered;
  ppg_absEma = (1.0 - ppg_ABS_EMA_ALPHA) * ppg_absEma + ppg_ABS_EMA_ALPHA * abs(ppg_latestFiltered);
  
  bool isPeak = (ppg_prev1 > ppg_prev2 && ppg_prev1 > ppg_curr && ppg_prev1 > (ppg_absEma * ppg_PEAK_THRESHOLD_RATIO) && (now - ppg_lastPeakTime > ppg_REFRACTORY_MS));

  if (isPeak && ppg_lastPeakTime > 0) {
    ppg_rawIbiMs = (float)(now - ppg_lastPeakTime);
    float peakAmp = ppg_prev1;
    
    if (ppg_rawIbiMs >= ppg_MIN_IBI_MS && ppg_rawIbiMs <= ppg_MAX_IBI_MS) {
      
      // SQI 평가 버퍼
      ppg_sqiIbiBuf[ppg_sqiIndex] = ppg_rawIbiMs; 
      ppg_sqiAmpBuf[ppg_sqiIndex] = abs(peakAmp);
      ppg_sqiIndex = (ppg_sqiIndex + 1) % ppg_SQI_BUF_SIZE;
      if (ppg_sqiCount < ppg_SQI_BUF_SIZE) ppg_sqiCount++;

      if (ppg_sqiCount >= 5) {
        float tempIbi[10], tempAmp[10];
        for (int i = 0; i < ppg_sqiCount; i++) {
          int idx = (ppg_sqiIndex - ppg_sqiCount + i + ppg_SQI_BUF_SIZE) % ppg_SQI_BUF_SIZE;
          tempIbi[i] = ppg_sqiIbiBuf[idx]; tempAmp[i] = ppg_sqiAmpBuf[idx];
        }
        float ibiCV = sys_calcCV(tempIbi, ppg_sqiCount);
        float ampCV = sys_calcCV(tempAmp, ppg_sqiCount);
        if (!isnan(ibiCV) && !isnan(ampCV)) {
          float ibiScore = constrain(100.0 * (1.0 - (ibiCV / 0.45)), 0.0, 100.0);
          float ampScore = constrain(100.0 * (1.0 - (ampCV / 1.20)), 0.0, 100.0);
          ppg_validWindowFlag = ((0.6 * ibiScore + 0.4 * ampScore) >= 50.0) ? 1 : 0;
        } else ppg_validWindowFlag = 0;
      }

      // Mean HR, SDNN, RMSSD 계산
      if (ppg_validWindowFlag == 1) {
        ppg_validIbiMs = ppg_rawIbiMs;
        ppg_ibiBuf[ppg_ibiIndex] = ppg_validIbiMs;
        ppg_ibiIndex = (ppg_ibiIndex + 1) % ppg_IBI_BUF_SIZE;
        if (ppg_ibiCount < ppg_IBI_BUF_SIZE) ppg_ibiCount++;

        if (ppg_ibiCount >= 5) {
          float temp[30];
          for (int i = 0; i < ppg_ibiCount; i++) {
            int idx = (ppg_ibiIndex - ppg_ibiCount + i + ppg_IBI_BUF_SIZE) % ppg_IBI_BUF_SIZE;
            temp[i] = ppg_ibiBuf[idx];
          }
          
          float meanIbi = sys_calcMean(temp, ppg_ibiCount);
          ppg_meanHR = (meanIbi > 0) ? (60000.0 / meanIbi) : NAN;
          ppg_sdnn = sys_calcStd(temp, ppg_ibiCount);
          
          float sumDiffSq = 0; int diffCount = 0;
          for (int i = 1; i < ppg_ibiCount; i++) {
            float diff = temp[i] - temp[i - 1];
            sumDiffSq += diff * diff;
            diffCount++;
          }
          ppg_rmssd = (diffCount > 0) ? sqrt(sumDiffSq / diffCount) : NAN;
        }

        // Calibration 카운트 누적
        if (mode == "CALIBRATION") {
          if (!isnan(ppg_meanHR)) sys_updateWelford(ppg_meanHR, ppg_baselineHRCount, ppg_meanHR_m, ppg_meanHR_s);
          if (!isnan(ppg_sdnn)) sys_updateWelford(ppg_sdnn, ppg_baselineSDNNCount, ppg_sdnn_m, ppg_sdnn_s);
          if (!isnan(ppg_rmssd)) sys_updateWelford(ppg_rmssd, ppg_baselineRMSSDCount, ppg_rmssd_m, ppg_rmssd_s);
          
          ppg_baselineMeanHRMean = ppg_meanHR_m; ppg_baselineSDNNMean = ppg_sdnn_m; ppg_baselineRMSSDMean = ppg_rmssd_m;
          if (ppg_baselineHRCount >= 2) ppg_baselineMeanHRStd = sqrt(ppg_meanHR_s / (ppg_baselineHRCount - 1));
          if (ppg_baselineSDNNCount >= 2) ppg_baselineSDNNStd = sqrt(ppg_sdnn_s / (ppg_baselineSDNNCount - 1));
          if (ppg_baselineRMSSDCount >= 2) ppg_baselineRMSSDStd = sqrt(ppg_rmssd_s / (ppg_baselineRMSSDCount - 1));
        }
      }
    }
  }
  if (isPeak) ppg_lastPeakTime = now;
}

void ppg_updateStressScore(String mode) {
  if (mode != "REST" && mode != "TRANSITION" && mode != "STRESS") return;

  // Mean HR은 심박수 확인용 원값으로만 사용하고, z-score 계산 및 최종 점수 반영에서 제외한다.

  if (ppg_baselineSDNNCount < 3 && ppg_baselineRMSSDCount < 3) return;

  ppg_sdnnStressZ = -sys_computeZScore(ppg_sdnn, ppg_baselineSDNNMean, ppg_baselineSDNNStd, 5.0);
  ppg_rmssdStressZ = -sys_computeZScore(ppg_rmssd, ppg_baselineRMSSDMean, ppg_baselineRMSSDStd, 5.0);

  float zSum = 0; int zCount = 0;
  if (!isnan(ppg_sdnnStressZ)) { zSum += ppg_sdnnStressZ; zCount++; }
  if (!isnan(ppg_rmssdStressZ)) { zSum += ppg_rmssdStressZ; zCount++; }
  ppg_finalStressZ = (zCount > 0) ? (zSum / zCount) : NAN;
}

// ==========================================
// 9. 통합 판단 및 출력
// ==========================================
void sys_updateCombinedStateAndPrint(unsigned long elapsedMs, String mode) {
  float validZSum = 0.0;
  sys_validFeatureCount = 0;
  bool labelMode = (mode == "REST" || mode == "STRESS");

  // Mean HR은 최종 점수에서 제외하므로 PPG 유효성 판단도 HR z-score에 의존하지 않는다.
  sys_ppgReadyForBle = labelMode && (ppg_validWindowFlag == 1) && (!isnan(ppg_sdnnStressZ) || !isnan(ppg_rmssdStressZ));
  bool gsrFresh = (millis() - gsr_latestScoreUpdateMs) <= (COMMON_FEATURE_WINDOW_MS + 10000UL);
  sys_gsrReadyForBle = labelMode && (gsr_latestMode == mode) && (gsr_contactOk == 1) && gsrFresh && (gsr_featureReady == 1);

  auto addValidZ = [&](float val) { if (!isnan(val)) { validZSum += val; sys_validFeatureCount++; } };

  if (sys_ppgReadyForBle) {
    // 최종 통합 스트레스 점수에는 Mean HR 제외, HRV 지표 2개만 반영
    addValidZ(ppg_sdnnStressZ);
    addValidZ(ppg_rmssdStressZ);
  }
  if (sys_gsrReadyForBle) {
    addValidZ(gsr_sclStressZ);
    if (gsr_latestScrAmplitudeMean30s > 0.0) addValidZ(gsr_scrAmplitudeStressZ);
    if (gsr_latestNsScrFrequencyPerMin > 0.0) addValidZ(gsr_nsScrFrequencyStressZ);
  }

  const int MIN_VALID_FEATURE_COUNT_FOR_DECISION = 3;
  if (sys_validFeatureCount >= MIN_VALID_FEATURE_COUNT_FOR_DECISION) {
    sys_integratedStressZ = validZSum / sys_validFeatureCount;
    sys_integratedStressState = classifyBinaryState(sys_integratedStressZ);
  } else {
    sys_integratedStressZ = NAN; sys_integratedStressState = "";
  }

  if (ENABLE_SERIAL_CSV) {
    if (!sys_csvHeaderPrinted) {
      Serial.println("timestamp,mode,raw_gsr,ir_raw,filtered_ppg,ppg_mean_hr,ppg_sdnn,ppg_rmssd,gsr_scl,gsr_scr_amp,gsr_scr_freq,ppg_sdnn_z,ppg_rmssd_z,gsr_scl_z,gsr_scr_amp_z,gsr_scr_freq_z,final_ppg_stress_z,final_gsr_stress_z,valid_feature_count,integrated_stress_z,integrated_state");
      sys_csvHeaderPrinted = true;
    }
    Serial.print(elapsedMs); Serial.print(","); printCsvString(mode);
    Serial.print(gsr_latestRawAdc); Serial.print(","); Serial.print(ppg_latestIrRaw); Serial.print(",");
    printCsvFloat(ppg_latestFiltered, 2);
    printCsvFloat(ppg_meanHR, 0); printCsvFloat(ppg_sdnn, 1); printCsvFloat(ppg_rmssd, 1);
    printCsvFloat(gsr_latestSclMean30s, 1); printCsvFloat(gsr_latestScrAmplitudeMean30s, 1); printCsvFloat(gsr_latestNsScrFrequencyPerMin, 1);
    // 각 특징값별 z-score 출력
    // Mean HR은 심박수 확인용 원값으로만 저장하고, z-score 출력에서는 제외한다.
    printCsvFloat(ppg_sdnnStressZ, 2); printCsvFloat(ppg_rmssdStressZ, 2);
    printCsvFloat(gsr_sclStressZ, 2); printCsvFloat(gsr_scrAmplitudeStressZ, 2); printCsvFloat(gsr_nsScrFrequencyStressZ, 2);
    printCsvFloat(ppg_finalStressZ, 2); printCsvFloat(gsr_finalStressZ, 2);
    Serial.print(sys_validFeatureCount); Serial.print(",");
    printCsvFloat(sys_integratedStressZ, 2); Serial.println(sys_integratedStressState);
  }
}

void sys_notifyBleFeatureData(String mode) {
  uint8_t flags = 0;
  if (sys_ppgReadyForBle) flags |= 0x01;
  if (sys_gsrReadyForBle) flags |= 0x02;
  if (mode == "REST" || mode == "STRESS") flags |= 0x04;
  if (sys_measurementActive) flags |= 0x08;

  ble_latestFeatures.seq = ble_seq++;
  ble_latestFeatures.mode_code = getModeCode(mode);
  ble_latestFeatures.flags = flags;
  ble_latestFeatures.mean_hr = isnan(ppg_meanHR) ? -1.0 : round(ppg_meanHR);
  ble_latestFeatures.final_ppg_stress_z = isnan(ppg_finalStressZ) ? -1.0 : ppg_finalStressZ;
  ble_latestFeatures.final_gsr_stress_z = isnan(gsr_finalStressZ) ? -1.0 : gsr_finalStressZ;
  ble_latestFeatures.integrated_stress_z = isnan(sys_integratedStressZ) ? -1.0 : sys_integratedStressZ;
  
  if (sys_integratedStressState == "Rest") ble_latestFeatures.integrated_level = 1;
  else if (sys_integratedStressState == "Stress") ble_latestFeatures.integrated_level = 2;
  else ble_latestFeatures.integrated_level = 0;

  featureDataChar.writeValue((uint8_t*)&ble_latestFeatures, sizeof(ble_latestFeatures));
}

// ==========================================
// 10. 메인 루프
// ==========================================
void setup() {
  Serial.begin(115200); Wire.begin();
  if (ppg_sensor.begin(Wire, I2C_SPEED_FAST)) { 
    ppg_sensor.setup(0x5F, 4, 2, 100, 411, 16384); 
    ppg_sensor.setPulseAmplitudeRed(0x5F); 
    ppg_sensor.setPulseAmplitudeIR(0x5F); 
  }
  if (BLE.begin()) { 
    BLE.setLocalName("BioGrip_Feature"); 
    BLE.setAdvertisedService(bioDataService); 
    bioDataService.addCharacteristic(featureDataChar); 
    bioDataService.addCharacteristic(controlPointChar); 
    BLE.addService(bioDataService); 
    BLE.advertise(); 
  }
}

void startMeasurement() { 
  sys_measurementActive = true; 
  sys_resetAll(); 
  unsigned long t0 = millis(); 
  ppg_startTimeMs = t0; 
  gsr_startTimeMs = t0; 
  ble_seq = 0; 
  ble_lastNotifyTime = t0; 
}

void loop() {
  BLE.poll();
  if (controlPointChar.written()) { 
    if (controlPointChar.value() == 1) startMeasurement(); 
    else if (controlPointChar.value() == 2) sys_measurementActive = false; 
  }
  if (Serial.available()) { 
    char cmd = Serial.read(); 
    if (cmd == 'S' || cmd == 's') startMeasurement(); 
    else if (cmd == 'X' || cmd == 'x') sys_measurementActive = false; 
  }

  if (!sys_measurementActive) return;
  unsigned long now = millis(); 
  unsigned long elapsedMs = now - ppg_startTimeMs;
  bool noFinger = (ppg_consecutiveNoFinger >= ppg_NO_FINGER_CONFIRM_COUNT);
  String mode = getMode(elapsedMs, noFinger);

  if (now - gsr_lastSampleTime >= GSR_SAMPLE_INTERVAL_MS) { 
    gsr_lastSampleTime = now; 
    if (!noFinger) gsr_processSample(elapsedMs, mode); 
  }
  
  if (now - ppg_lastSampleTime >= PPG_SAMPLE_INTERVAL_MS) { 
    ppg_lastSampleTime = now; 
    ppg_processSample(now, elapsedMs, mode); 
  }

  static unsigned long lastPrintTime = 0;
  if (now - lastPrintTime >= 1000UL) {
    lastPrintTime = now;
    ppg_updateStressScore(mode);
    sys_updateCombinedStateAndPrint(elapsedMs, mode);
  }
  
  if (now - ble_lastNotifyTime >= BLE_NOTIFY_INTERVAL_MS) {
    ble_lastNotifyTime = now;
    sys_notifyBleFeatureData(mode);
  }
}