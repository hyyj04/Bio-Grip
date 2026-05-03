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

// 손가락 미접촉 판단 기준
// 사용하는 센서 raw 값 범위에 따라 조정 가능
const long NO_FINGER_IR_THRESHOLD = 10000;

// ===============================
// 2. Butterworth 필터 계수
// 기존에 계산했던 HPF 0.5 Hz, LPF 8 Hz, Fs 100 Hz
// ===============================

// HPF
float b_hp[3] = {0.97803048, -1.95606096, 0.97803048};
float a_hp[3] = {1.0, -1.95557824, 0.95654368};

// LPF
float b_lp[3] = {0.0461318, 0.0922636, 0.0461318};
float a_lp[3] = {1.0, -1.30728503, 0.49181224};

// 필터 상태 변수
float hp_x1 = 0, hp_x2 = 0;
float hp_y1 = 0, hp_y2 = 0;

float lp_x1 = 0, lp_x2 = 0;
float lp_y1 = 0, lp_y2 = 0;

// ===============================
// 3. 피크 검출 및 IBI 설정
// ===============================
float ppg_prev2 = 0;
float ppg_prev1 = 0;
float ppg_curr = 0;

float abs_ema = 0;
const float ABS_EMA_ALPHA = 0.01;
const float PEAK_THRESHOLD_RATIO = 0.35;

unsigned long lastPeakTime = 0;
const unsigned long REFRACTORY_MS = 300;

float lastIbiMs = NAN;
float currentIbiMs = NAN;
float validIbiMs = NAN;

// IBI 유효 범위
const float MIN_IBI_MS = 300.0;
const float MAX_IBI_MS = 1500.0;

// local median 기반 이상치 제거
const int MEDIAN_BUF_SIZE = 7;
float medianBuf[MEDIAN_BUF_SIZE];
int medianCount = 0;
int medianIndex = 0;

const float LOCAL_MEDIAN_TOLERANCE = 0.30;

// ===============================
// 4. HRV 계산용 IBI 버퍼
// ===============================
const int IBI_BUF_SIZE = 30;
float ibiBuf[IBI_BUF_SIZE];
int ibiCount = 0;
int ibiIndex = 0;

float meanHR = NAN;
float sdnn = NAN;
float rmssd = NAN;

// ===============================
// 5. baseline 계산용 누적 변수
// ===============================
int baselineMetricCount = 0;

float baselineMeanHRMean = NAN;
float baselineMeanHRStd = NAN;

float baselineSDNNMean = NAN;
float baselineSDNNStd = NAN;

float baselineRMSSDMean = NAN;
float baselineRMSSDStd = NAN;

// Welford 누적 변수
float meanHR_m = 0, meanHR_s = 0;
float sdnn_m = 0, sdnn_s = 0;
float rmssd_m = 0, rmssd_s = 0;

// 카운터
int noFingerCount = 0;

// ===============================
// 6. 필터 함수
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
// 7. 배열 관련 함수
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

// ===============================
// 8. IBI 이상치 검사
// ===============================
bool isValidIbi(float ibi) {
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

// ===============================
// 9. Mean HR, SDNN, RMSSD 계산
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
// 10. baseline 누적 계산
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

// ===============================
// 11. mode 반환
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
// 12. setup
// ===============================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR,MAX30102_NOT_FOUND");
    while (1);
  }

  // MAX30102 설정
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

  Serial.println("time_ms,mode,ir_raw,filtered_ppg,ibi_ms,valid_ibi_ms,mean_hr,sdnn,rmssd,baseline_mean_hr_mean,baseline_mean_hr_std,baseline_sdnn_mean,baseline_sdnn_std,baseline_rmssd_mean,baseline_rmssd_std,ibi_count,baseline_count,no_finger_count");
}

// ===============================
// 13. loop
// ===============================
void loop() {
  unsigned long now = millis();

  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) {
    return;
  }

  lastSampleTime = now;

  unsigned long elapsedMs = now - startTime;

  long irRaw = particleSensor.getIR();

  bool noFinger = irRaw < NO_FINGER_IR_THRESHOLD;

  if (noFinger) {
    noFingerCount++;
  }

  String mode = getMode(elapsedMs, noFinger);

  float filteredPpg = applyBandpass((float)irRaw);

  // 피크 검출은 손가락이 있고 안정화 이후부터 수행
  if (!noFinger && mode != "STABILIZING") {
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
        currentIbiMs = (float)(now - lastPeakTime);

        if (isValidIbi(currentIbiMs)) {
          validIbiMs = currentIbiMs;
          addMedianBuffer(validIbiMs);
          addIbiBuffer(validIbiMs);
          updateHrvMetrics();
        } else {
          validIbiMs = NAN;
        }
      }

      lastPeakTime = now;
    }
  }

  // calibration 구간에서는 baseline 누적
  if (mode == "CALIBRATION") {
    updateBaselineMetrics();
  }

  // 1초마다 CSV 형식으로 출력
  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;

    Serial.print(elapsedMs);
    Serial.print(",");
    Serial.print(mode);
    Serial.print(",");
    Serial.print(irRaw);
    Serial.print(",");
    Serial.print(filteredPpg, 4);
    Serial.print(",");

    if (isnan(currentIbiMs)) Serial.print("");
    else Serial.print(currentIbiMs, 2);
    Serial.print(",");

    if (isnan(validIbiMs)) Serial.print("");
    else Serial.print(validIbiMs, 2);
    Serial.print(",");

    if (isnan(meanHR)) Serial.print("");
    else Serial.print(meanHR, 2);
    Serial.print(",");

    if (isnan(sdnn)) Serial.print("");
    else Serial.print(sdnn, 2);
    Serial.print(",");

    if (isnan(rmssd)) Serial.print("");
    else Serial.print(rmssd, 2);
    Serial.print(",");

    if (isnan(baselineMeanHRMean)) Serial.print("");
    else Serial.print(baselineMeanHRMean, 2);
    Serial.print(",");

    if (isnan(baselineMeanHRStd)) Serial.print("");
    else Serial.print(baselineMeanHRStd, 2);
    Serial.print(",");

    if (isnan(baselineSDNNMean)) Serial.print("");
    else Serial.print(baselineSDNNMean, 2);
    Serial.print(",");

    if (isnan(baselineSDNNStd)) Serial.print("");
    else Serial.print(baselineSDNNStd, 2);
    Serial.print(",");

    if (isnan(baselineRMSSDMean)) Serial.print("");
    else Serial.print(baselineRMSSDMean, 2);
    Serial.print(",");

    if (isnan(baselineRMSSDStd)) Serial.print("");
    else Serial.print(baselineRMSSDStd, 2);
    Serial.print(",");

    Serial.print(ibiCount);
    Serial.print(",");
    Serial.print(baselineMetricCount);
    Serial.print(",");
    Serial.println(noFingerCount);
  }
}