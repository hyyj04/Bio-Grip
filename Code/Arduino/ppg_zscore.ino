#include <Wire.h>
#include "MAX30105.h"

MAX30105 particleSensor;


// ===============================
// 1. 샘플링 및 구간 설정
// ===============================
const float FS = 100.0;
const unsigned long SAMPLE_INTERVAL_MS = 10;

const unsigned long STABILIZING_TIME_MS = 5000;
const unsigned long CALIBRATION_TIME_MS = 60000;

unsigned long startTime = 0;
unsigned long lastSampleTime = 0;
unsigned long lastPrintTime = 0;

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
const float MIN_SQI_SCORE = 45.0;

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

const int MIN_BASELINE_METRIC_COUNT_FOR_Z = 10;


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
  // MONITORING 구간에서만 z-score 기반 최종 점수 계산
  if (mode != "MONITORING") {
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
    isnan(baselineRMSSDStd) ||
    baselineMeanHRStd <= 0 ||
    baselineSDNNStd <= 0 ||
    baselineRMSSDStd <= 0
  ) {
    resetPpgStressScore();
    return;
  }

  // z-score 계산
  // Mean HR: 증가 방향이 스트레스
  meanHrZ = (meanHR - baselineMeanHRMean) / baselineMeanHRStd;

  // SDNN, RMSSD: 감소 방향이 스트레스
  sdnnZ = (sdnn - baselineSDNNMean) / baselineSDNNStd;
  rmssdZ = (rmssd - baselineRMSSDMean) / baselineRMSSDStd;

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

  if (elapsedMs < STABILIZING_TIME_MS) {
    return "STABILIZING";
  }

  if (elapsedMs < STABILIZING_TIME_MS + CALIBRATION_TIME_MS) {
    return "CALIBRATION";
  }

  return "MONITORING";
}


// ===============================
// 20. setup
// ===============================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR,MAX30102_NOT_FOUND");
    while (1);
  }

  byte ledBrightness = 0x1F;
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

  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);

  startTime = millis();
  lastSampleTime = millis();
  lastPrintTime = millis();

  Serial.println(
    "time_ms,mode,ir_raw,filtered_ppg,ibi_ms,valid_ibi_ms,"
    "mean_hr,sdnn,rmssd,"
    "baseline_mean_hr_mean,baseline_mean_hr_std,"
    "baseline_sdnn_mean,baseline_sdnn_std,"
    "baseline_rmssd_mean,baseline_rmssd_std,"
    "baseline_ibi_mean,baseline_ibi_std,"
    "sqi_score,valid_window_flag,"
    "mean_hr_z,sdnn_z,rmssd_z,"
    "mean_hr_stress_score,sdnn_stress_score,rmssd_stress_score,"
    "ppg_avg_score,hrv_score,final_ppg_stress_score,final_ppg_stress_level,"
    "ibi_count,baseline_count,no_finger_count"
  );
}


// ===============================
// 21. loop
// ===============================
void loop() {
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

      bool isPeak = false;

      if (
        ppg_prev1 > ppg_prev2 &&
        ppg_prev1 > ppg_curr &&
        ppg_prev1 > peakThreshold &&
        now - lastPeakTime > REFRACTORY_MS
      ) {
        isPeak = true;
      }

      if (isPeak) {
        if (lastPeakTime > 0) {
          rawIbiMs = (float)(now - lastPeakTime);
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

        lastPeakTime = now;
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

    Serial.print(elapsedMs);
    Serial.print(",");

    Serial.print(mode);
    Serial.print(",");

    Serial.print(irRaw);
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

    Serial.println(noFingerCount);

    // 출력 후 다음 1초 구간의 새 IBI만 표시하기 위해 리셋
    rawIbiMs = NAN;
    validIbiMs = NAN;
  }
}