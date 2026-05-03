#include <Wire.h>
#include "MAX30105.h"

MAX30105 particleSensor;

// ===============================
// 1. 샘플링 및 구간 설정
// ===============================
const float FS = 100.0; // 샘플링 주파수 (Hz). 1초에 100번 샘플링.
const unsigned long SAMPLE_INTERVAL_MS = 10; // 샘플링 간격 (ms). 1000ms / 100Hz = 10ms.

const unsigned long STABILIZING_TIME_MS = 5000; // 5초. 측정 시작 후 신호 안정화 대기 시간.
const unsigned long CALIBRATION_TIME_MS = 60000; // 60초. 사용자의 평상시 HRV(Baseline)를 측정하는 시간.

unsigned long startTime = 0; // 프로그램 시작 시간
unsigned long lastSampleTime = 0; // 마지막으로 센서 값을 읽은 시간
unsigned long lastPrintTime = 0; // 마지막으로 시리얼 출력을 한 시간

const unsigned long PRINT_INTERVAL_MS = 1000; // 1초. 시리얼 모니터에 데이터를 출력하는 간격.

// 손가락 미접촉 판단 기준. IR(적외선) 센서의 원시 값이 이 값보다 작으면 손가락이 없는 것으로 판단.
// 센서의 특성이나 주변 환경에 따라 조정이 필요할 수 있습니다.
const long NO_FINGER_IR_THRESHOLD = 10000; 

// ===============================
// 2. Butterworth 필터 계수
// 기존에 계산했던 HPF 0.5 Hz, LPF 8 Hz, Fs 100 Hz
// ===============================

// HPF
float b_hp[3] = {0.97803048, -1.95606096, 0.97803048}; // 고역 통과 필터(HPF)의 분자(b) 계수
float a_hp[3] = {1.0, -1.95557824, 0.95654368};       // 고역 통과 필터(HPF)의 분모(a) 계수

// LPF
float b_lp[3] = {0.0461318, 0.0922636, 0.0461318}; // 저역 통과 필터(LPF)의 분자(b) 계수
float a_lp[3] = {1.0, -1.30728503, 0.49181224};       // 저역 통과 필터(LPF)의 분모(a) 계수

// 필터 상태 변수 (IIR 필터는 이전 입/출력 값을 사용하므로 상태를 저장해야 함)
float hp_x1 = 0, hp_x2 = 0; // HPF의 이전 입력 값 (x[n-1], x[n-2])
float hp_y1 = 0, hp_y2 = 0; // HPF의 이전 출력 값 (y[n-1], y[n-2])

float lp_x1 = 0, lp_x2 = 0; // LPF의 이전 입력 값 (x[n-1], x[n-2])
float lp_y1 = 0, lp_y2 = 0; // LPF의 이전 출력 값 (y[n-1], y[n-2])

// ===============================
// 3. 피크 검출 및 IBI 설정
// ===============================
float ppg_prev2 = 0; // 2 샘플 이전의 필터링된 PPG 값
float ppg_prev1 = 0; // 1 샘플 이전의 필터링된 PPG 값
float ppg_curr = 0;  // 현재 필터링된 PPG 값

float abs_ema = 0; // 필터링된 PPG 신호의 절대값에 대한 지수이동평균(EMA). 동적 임계값 계산에 사용.
const float ABS_EMA_ALPHA = 0.01; // EMA 계산에 사용되는 평활 계수
const float PEAK_THRESHOLD_RATIO = 0.35; // EMA 값에 이 비율을 곱하여 피크 검출 임계값을 정함.

unsigned long lastPeakTime = 0; // 마지막으로 피크가 검출된 시간
const unsigned long REFRACTORY_MS = 300; // 불응기(ms). 한 번 피크를 찾은 후 이 시간 동안은 다음 피크를 찾지 않음. (심박수 200bpm에 해당)

float lastIbiMs = NAN;    // 이전 IBI (Inter-Beat Interval, 심박 간격) 값
float currentIbiMs = NAN; // 현재 계산된 IBI 값 (이상치 제거 전)
float validIbiMs = NAN;   // 이상치 검사를 통과한 유효한 IBI 값

// IBI 유효 범위 (생리학적 범위). 심박수 40bpm(1500ms) ~ 200bpm(300ms)에 해당.
const float MIN_IBI_MS = 300.0; 
const float MAX_IBI_MS = 1500.0;

// Local Median 기반 이상치 제거를 위한 설정
const int MEDIAN_BUF_SIZE = 7; // 최근 IBI 값을 저장할 버퍼 크기
float medianBuf[MEDIAN_BUF_SIZE]; // 중간값 계산용 순환 버퍼
int medianCount = 0; // 버퍼에 저장된 IBI 개수
int medianIndex = 0; // 버퍼의 현재 인덱스

const float LOCAL_MEDIAN_TOLERANCE = 0.30; // 중간값에서 ±30%를 벗어나면 이상치로 판단

// ===============================
// 4. HRV 계산용 IBI 버퍼
// ===============================
const int IBI_BUF_SIZE = 30; // HRV 지표 계산에 사용할 IBI 개수 (약 30초 분량)
float ibiBuf[IBI_BUF_SIZE]; // 유효 IBI를 저장하는 순환 버퍼
int ibiCount = 0; // 버퍼에 저장된 IBI 개수
int ibiIndex = 0; // 버퍼의 현재 인덱스

float meanHR = NAN; // 평균 심박수 (Mean Heart Rate)
float sdnn = NAN;   // SDNN (Standard deviation of NN intervals), 전체 IBI의 표준편차
float rmssd = NAN;  // RMSSD (Root mean square of successive differences), 연속된 IBI 차이의 제곱평균제곱근

// ===============================
// 5. baseline 계산용 누적 변수
// ===============================
int baselineMetricCount = 0; // Baseline 계산에 사용된 HRV 지표의 개수

// Baseline 기간 동안 계산된 각 HRV 지표의 평균과 표준편차
float baselineMeanHRMean = NAN;
float baselineMeanHRStd = NAN;

float baselineSDNNMean = NAN;
float baselineSDNNStd = NAN;

float baselineRMSSDMean = NAN;
float baselineRMSSDStd = NAN;

// Welford's algorithm을 사용하여 온라인으로 평균/분산을 계산하기 위한 변수들
// m: 평균, s: 제곱합의 합
float meanHR_m = 0, meanHR_s = 0; 
float sdnn_m = 0, sdnn_s = 0;
float rmssd_m = 0, rmssd_s = 0;

int noFingerCount = 0; // 손가락이 감지되지 않은 샘플의 수

// ===============================
// 6. 필터 함수
// ===============================
/**
 * @brief 입력 신호 x에 고역 통과 필터(HPF)를 적용합니다.
 * @param x 필터링할 원시 신호 샘플
 * @return 필터링된 신호 샘플
 */
float applyHPF(float x) {
  float y = b_hp[0] * x + b_hp[1] * hp_x1 + b_hp[2] * hp_x2
            - a_hp[1] * hp_y1 - a_hp[2] * hp_y2;

  hp_x2 = hp_x1;
  hp_x1 = x;
  hp_y2 = hp_y1;
  hp_y1 = y;

  return y;
}

/**
 * @brief 입력 신호 x에 저역 통과 필터(LPF)를 적용합니다.
 * @param x 필터링할 원시 신호 샘플
 * @return 필터링된 신호 샘플
 */
float applyLPF(float x) {
  float y = b_lp[0] * x + b_lp[1] * lp_x1 + b_lp[2] * lp_x2
            - a_lp[1] * lp_y1 - a_lp[2] * lp_y2;

  lp_x2 = lp_x1;
  lp_x1 = x;
  lp_y2 = lp_y1;
  lp_y1 = y;

  return y;
}

/**
 * @brief 입력 신호 x에 대역 통과 필터(Bandpass Filter)를 적용합니다.
 * @param x 필터링할 원시 신호 샘플
 * @return 필터링된 신호 샘플
 */
float applyBandpass(float x) {
  float hp = applyHPF(x);
  float lp = applyLPF(hp);
  return lp;
}

// ===============================
// 7. 배열 관련 함수
// ===============================
/**
 * @brief float 배열의 평균을 계산합니다.
 * @param arr 평균을 계산할 배열
 * @param n 배열의 크기
 * @return 계산된 평균값
 */
float calcMean(float arr[], int n) {
  if (n <= 0) return NAN;

  float sum = 0;
  for (int i = 0; i < n; i++) {
    sum += arr[i];
  }
  return sum / n;
}

/**
 * @brief float 배열의 표본 표준편차를 계산합니다.
 * @param arr 표준편차를 계산할 배열
 * @param n 배열의 크기
 * @return 계산된 표준편차 값
 */
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

/**
 * @brief float 배열의 중간값을 계산합니다.
 * @param arr 중간값을 계산할 배열
 * @param n 배열의 크기
 * @return 계산된 중간값
 */
float calcMedian(float arr[], int n) {
  if (n <= 0) return NAN;

  // 원본 배열을 훼손하지 않기 위해 임시 배열에 복사
  float temp[MEDIAN_BUF_SIZE];

  for (int i = 0; i < n; i++) {
    temp[i] = arr[i];
  }

  // 간단한 선택 정렬로 배열을 오름차순 정렬
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (temp[j] < temp[i]) {
        float t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }

  // 배열 크기에 따라 중간값 계산
  if (n % 2 == 1) {
    return temp[n / 2];
  } else {
    return (temp[n / 2 - 1] + temp[n / 2]) / 2.0;
  }
}

// ===============================
// 8. IBI 이상치 검사
// ===============================
/**
 * @brief 주어진 IBI 값이 유효한지 검사합니다.
 * @param ibi 검사할 IBI 값 (ms)
 * @return 유효하면 true, 아니면 false
 */
bool isValidIbi(float ibi) {
  if (isnan(ibi)) return false;

  // 1. 생리학적 범위를 벗어나는지 확인 (심박수 40~200bpm)
  if (ibi < MIN_IBI_MS || ibi > MAX_IBI_MS) {
    return false;
  }

  // 2. Local Median을 기준으로 갑자기 튀는 값인지 확인
  if (medianCount >= 3) {
    float med = calcMedian(medianBuf, medianCount);
    float lower = med * (1.0 - LOCAL_MEDIAN_TOLERANCE);
    float upper = med * (1.0 + LOCAL_MEDIAN_TOLERANCE);

    if (ibi < lower || ibi > upper) {
      return false; // 최근 IBI들의 중간값에서 30% 이상 벗어나면 이상치로 간주
    }
  }

  return true;
}

void addMedianBuffer(float ibi) {
  /**
   * @brief 중간값 계산용 순환 버퍼에 IBI 값을 추가합니다.
   * @param ibi 추가할 IBI 값
   */
  medianBuf[medianIndex] = ibi;
  medianIndex = (medianIndex + 1) % MEDIAN_BUF_SIZE;

  if (medianCount < MEDIAN_BUF_SIZE) {
    medianCount++;
  }
}

void addIbiBuffer(float ibi) {
  /**
   * @brief HRV 계산용 순환 버퍼에 IBI 값을 추가합니다.
   * @param ibi 추가할 IBI 값
   */
  ibiBuf[ibiIndex] = ibi;
  ibiIndex = (ibiIndex + 1) % IBI_BUF_SIZE;

  if (ibiCount < IBI_BUF_SIZE) {
    ibiCount++;
  }
}

// ===============================
// 9. Mean HR, SDNN, RMSSD 계산
// ===============================
/**
 * @brief IBI 버퍼의 데이터를 사용하여 HRV 지표(Mean HR, SDNN, RMSSD)를 업데이트합니다.
 */
void updateHrvMetrics() {
  if (ibiCount < 5) {
    meanHR = NAN;
    sdnn = NAN;
    rmssd = NAN;
    return;
  }

  float temp[IBI_BUF_SIZE];
  
  // 순환 버퍼(ibiBuf)의 데이터를 계산하기 쉽도록 임시 선형 배열(temp)로 복사
  for (int i = 0; i < ibiCount; i++) {
    int idx = (ibiIndex - ibiCount + i + IBI_BUF_SIZE) % IBI_BUF_SIZE;
    temp[i] = ibiBuf[idx];
  }

  // Mean HR 계산
  float meanIbi = calcMean(temp, ibiCount);
  meanHR = 60000.0 / meanIbi;

  // SDNN 계산
  sdnn = calcStd(temp, ibiCount);

  // RMSSD 계산
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
/**
 * @brief Welford's algorithm을 사용하여 온라인으로 평균과 분산을 업데이트합니다.
 * @param x 새로 추가할 데이터 포인트
 * @param n 현재까지의 데이터 개수 (새 데이터 포함)
 * @param m 업데이트될 평균 (참조 전달)
 * @param s 업데이트될 제곱합의 합 (참조 전달)
 */
void updateWelford(float x, int n, float &m, float &s) {
  float delta = x - m;
  m += delta / n;
  float delta2 = x - m;
  s += delta * delta2;
}

/**
 * @brief CALIBRATION 모드에서 Baseline HRV 지표(평균, 표준편차)를 누적 계산합니다.
 */
void updateBaselineMetrics() {
  if (isnan(meanHR) || isnan(sdnn) || isnan(rmssd)) {
    return;
  }

  baselineMetricCount++;

  // 각 HRV 지표에 대해 Welford 알고리즘을 적용하여 평균과 분산을 업데이트
  updateWelford(meanHR, baselineMetricCount, meanHR_m, meanHR_s);
  updateWelford(sdnn, baselineMetricCount, sdnn_m, sdnn_s);
  updateWelford(rmssd, baselineMetricCount, rmssd_m, rmssd_s);

  baselineMeanHRMean = meanHR_m;
  baselineSDNNMean = sdnn_m;
  baselineRMSSDMean = rmssd_m;

  // 분산(표준편차의 제곱) 계산
  if (baselineMetricCount >= 2) {
    baselineMeanHRStd = sqrt(meanHR_s / (baselineMetricCount - 1));
    baselineSDNNStd = sqrt(sdnn_s / (baselineMetricCount - 1));
    baselineRMSSDStd = sqrt(rmssd_s / (baselineMetricCount - 1));
  }
}

// ===============================
// 11. mode 반환
// ===============================
/**
 * @brief 현재 경과 시간과 손가락 접촉 여부에 따라 현재 동작 모드를 반환합니다.
 * @param elapsedMs 프로그램 시작 후 경과 시간 (ms)
 * @param noFinger 손가락 미접촉 여부
 * @return 현재 모드를 나타내는 문자열 ("NO_FINGER", "STABILIZING", "CALIBRATION", "MONITORING")
 */
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

  // I2C 통신 및 센서 초기화
  Wire.begin();

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR,MAX30102_NOT_FOUND");
    while (1);
  }

  // MAX30102/MAX30105 센서 설정
  byte ledBrightness = 0x5F; //0x1F --> 0x5F 로 변경 (밝기 증가)
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

  startTime = millis();
  lastSampleTime = millis();
  lastPrintTime = millis();

  // 시리얼 플로터나 데이터 분석을 위해 CSV 헤더 출력
  Serial.println("time_ms,mode,ir_raw,filtered_ppg,ibi_ms,valid_ibi_ms,mean_hr,sdnn,rmssd,baseline_mean_hr_mean,baseline_mean_hr_std,baseline_sdnn_mean,baseline_sdnn_std,baseline_rmssd_mean,baseline_rmssd_std,ibi_count,baseline_count,no_finger_count");
}

// ===============================
// 13. loop
// ===============================
void loop() {
  unsigned long now = millis();

  // 설정된 샘플링 간격(10ms)마다 아래 로직을 실행
  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) {
    return;
  }

  lastSampleTime = now;

  unsigned long elapsedMs = now - startTime;

  // 센서로부터 IR 원시 데이터 읽기
  long irRaw = particleSensor.getIR();

  // 손가락 접촉 여부 판단
  bool noFinger = irRaw < NO_FINGER_IR_THRESHOLD;

  if (noFinger) {
    noFingerCount++;
  }

  // 현재 상태(모드) 결정
  String mode = getMode(elapsedMs, noFinger);

  // 원시 데이터에 대역 통과 필터 적용하여 PPG 신호 추출
  float filteredPpg = applyBandpass((float)irRaw);

  // 피크 검출은 손가락이 접촉되어 있고, 신호 안정화 기간이 지난 후에만 수행
  if (!noFinger && mode != "STABILIZING") {
    // 피크 검출을 위해 이전 2개의 PPG 샘플 값을 업데이트
    ppg_prev2 = ppg_prev1;
    ppg_prev1 = ppg_curr;
    ppg_curr = filteredPpg;

    abs_ema = (1.0 - ABS_EMA_ALPHA) * abs_ema + ABS_EMA_ALPHA * abs(filteredPpg);
    float peakThreshold = abs_ema * PEAK_THRESHOLD_RATIO;

    bool isPeak = false;

    // 피크 조건: (1) 현재 샘플(prev1)이 양 옆 샘플보다 크고, (2) 동적 임계값보다 크며, (3) 불응기 시간보다 오래 지났을 때
    if (
      ppg_prev1 > ppg_prev2 &&
      ppg_prev1 > ppg_curr &&
      ppg_prev1 > peakThreshold &&
      now - lastPeakTime > REFRACTORY_MS
    ) {
      isPeak = true;
    }

    if (isPeak) {
      // 이전 피크가 존재했다면, IBI(피크 간 시간 간격) 계산
      if (lastPeakTime > 0) {
        currentIbiMs = (float)(now - lastPeakTime);

        // 계산된 IBI가 유효한지 검사
        if (isValidIbi(currentIbiMs)) {
          validIbiMs = currentIbiMs;
          addMedianBuffer(validIbiMs);
          addIbiBuffer(validIbiMs);
          // 새로운 유효 IBI가 추가되었으므로 HRV 지표 업데이트
          updateHrvMetrics();
        } else {
          validIbiMs = NAN;
        }
      }

      lastPeakTime = now;
    }
  }

  // CALIBRATION 모드일 때, Baseline 지표를 누적 계산
  if (mode == "CALIBRATION") {
    updateBaselineMetrics();
  }

  // PRINT_INTERVAL_MS(1초)마다 모든 데이터를 CSV 형식으로 시리얼 출력
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