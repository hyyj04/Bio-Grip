/*
  GSR Signal Processing for Arduino IDE

  Version:
  Dual Baseline - SCL Mean / SCR Detection 분리

  Pipeline:
  1. Data Acquisition: 10 Hz
  2. 1st preprocessing: Median Filter, N = 3
  3. 2nd preprocessing (SCL용): Moving Average, N = 150 (15초)
  4. 2nd preprocessing (SCR검출용): EMA, alpha = 0.02 (τ ≈ 5초)
  5. Phasic separation: Phasic = medianGSR - scrDetectBaseline
  6. Baseline statistics: 15~60 sec
  7. Measurement: after 60 sec
  8. Feature extraction every 30 sec:
     - SCL Mean        (sclBaseline 기반)
     - SCR Amplitude Mean
     - NS-SCR Frequency
*/

#define GSR_PIN A0

// =====================
// CSV output mode
// =====================
// true  : 모든 10Hz 샘플을 CSV로 출력
// false : 30초 feature window 완료 시점에만 출력
const bool PRINT_EVERY_SAMPLE = true;

// =====================
// Sampling parameters
// =====================
const unsigned long SAMPLE_INTERVAL_MS = 100;   // 10 Hz
const int FS_GSR = 10;

// =====================
// Preprocessing parameters
// =====================
const int MEDIAN_N = 3;
const int MA_N = 150;     // 15 sec at 10 Hz (SCL Mean용)

// EMA alpha for SCR detection baseline
// τ = 1 / alpha = 50 samples = 5 sec at 10 Hz
// alpha가 클수록 baseline이 신호를 빠르게 따라가 phasic이 작아짐
// alpha가 작을수록 baseline이 느리게 따라가 phasic이 커짐
const float SCR_BASELINE_ALPHA = 0.02;

// =====================
// Time section parameters
// =====================
const int USER_WARMUP_SEC = 15;
const int BASELINE_END_SEC = 60;
const int FEATURE_WINDOW_SEC = 30;

const int MA_FILL_SAMPLES = MA_N;
const int USER_WARMUP_SAMPLES = USER_WARMUP_SEC * FS_GSR;

// warm-up은 최소한 Moving Average 버퍼가 한 번 채워질 시간 이상으로 보장
const int WARMUP_SAMPLES =
  (USER_WARMUP_SAMPLES > MA_FILL_SAMPLES) ? USER_WARMUP_SAMPLES : MA_FILL_SAMPLES;

const int BASELINE_END_SAMPLES = BASELINE_END_SEC * FS_GSR;
const int FEATURE_WINDOW_SAMPLES = FEATURE_WINDOW_SEC * FS_GSR;

// =====================
// SCR detection parameters
// =====================
// 경험적 상수
// baseline phasic 표준편차의 약 3배 이상을 SCR 후보로 판단하는 휴리스틱
// 안정/자극 상태 재측정 후 조정 가능
const float THRESHOLD_K = 3.0;

// baseline std가 지나치게 작을 때 임계값이 과도하게 낮아지는 것을 방지
const float MIN_SCR_THRESHOLD_ADC = 6.0;

// baseline std가 과도하게 커져 SCR 검출 임계값이 비현실적으로 상승하는 것을 방지
const float MAX_SCR_THRESHOLD_ADC = 15.0;

const int REFRACTORY_SAMPLES = 10;  // 1 sec at 10 Hz

// =====================
// Median filter variables
// =====================
float medBuf[MEDIAN_N] = {0, 0, 0};
int medIndex = 0;
bool medFilled = false;

// =====================
// SCL용 Moving average variables
// Circular buffer
// =====================
float maBuf[MA_N];
int maIndex = 0;
int maCount = 0;
float maSum = 0.0;

// =====================
// SCR 검출용 EMA baseline variables
// =====================
float scrDetectBaseline = 0.0;
bool scrBaselineInitialized = false;

// =====================
// Baseline statistics
// EMA 기반 phasic (unclipped) 기준
// =====================
float baselinePhasicSum = 0.0;
float baselinePhasicSqSum = 0.0;
int baselineCount = 0;

float baselinePhasicMean = 0.0;
float baselinePhasicStd = 0.0;
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

// SCR 피크가 실제로 발생한 샘플/시간 기록용
int detectedScrSampleIndex = -1;
unsigned long detectedScrTimeMs = 0;

// 샘플 시간 기록용
unsigned long prev2SampleTimeMs = 0;
unsigned long prev1SampleTimeMs = 0;
unsigned long currSampleTimeMs = 0;

// =====================
// General variables
// =====================
unsigned long lastSampleTime = 0;
int sampleIndex = 0;


// =====================
// Median of 3
// =====================
float median3(float a, float b, float c) {
  if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
  if ((b >= a && b <= c) || (b <= a && b >= c)) return b;
  return c;
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

  // 초기 3개가 다 채워지기 전에는 raw 값을 그대로 사용
  if (!medFilled) {
    return x;
  }

  return median3(medBuf[0], medBuf[1], medBuf[2]);
}


// =====================
// SCL용 Moving average update
// Circular buffer
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
// SCR 검출용 EMA baseline update
// 첫 샘플은 raw값으로 초기화하여 cold-start 왜곡 방지
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
  if (baselineCount <= 1) {
    baselinePhasicMean = 0.0;
    baselinePhasicStd = 0.0;
    scrThreshold = MIN_SCR_THRESHOLD_ADC;
    baselineReady = true;
    return;
  }

  baselinePhasicMean = baselinePhasicSum / baselineCount;

  float variance =
    (baselinePhasicSqSum / baselineCount) -
    (baselinePhasicMean * baselinePhasicMean);

  if (variance < 0.0) {
    variance = 0.0;
  }

  baselinePhasicStd = sqrt(variance);

  // Adaptive threshold
  scrThreshold = THRESHOLD_K * baselinePhasicStd;

  // Lower bound
  if (scrThreshold < MIN_SCR_THRESHOLD_ADC) {
    scrThreshold = MIN_SCR_THRESHOLD_ADC;
  }

  // Upper bound
  if (scrThreshold > MAX_SCR_THRESHOLD_ADC) {
    scrThreshold = MAX_SCR_THRESHOLD_ADC;
  }

  baselineReady = true;
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
    "baseline_phasic_mean,"
    "baseline_phasic_std,"
    "feature_ready,"
    "min_scr_threshold_adc,"
    "max_scr_threshold_adc"
  );
}


// =====================
// CSV row print
// =====================
void printCsvRow(
  unsigned long timeMs,
  int sIndex,
  int mode,
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
  int featureReady
) {
  Serial.print(timeMs);
  Serial.print(",");
  Serial.print(sIndex);
  Serial.print(",");
  Serial.print(mode);
  Serial.print(",");
  Serial.print(rawGsr);
  Serial.print(",");
  Serial.print(medianGsr, 2);
  Serial.print(",");
  Serial.print(sclBaseline, 2);
  Serial.print(",");
  Serial.print(scrDetectBl, 2);
  Serial.print(",");
  Serial.print(phasic, 2);
  Serial.print(",");
  Serial.print(phasicPos, 2);
  Serial.print(",");
  Serial.print(scrThreshold, 2);
  Serial.print(",");
  Serial.print(scrFlag);
  Serial.print(",");
  Serial.print(scrPeakTimeMs);
  Serial.print(",");
  Serial.print(scrPeakSampleIndex);
  Serial.print(",");
  Serial.print(sclMean30s, 2);
  Serial.print(",");
  Serial.print(scrAmplitudeMean30s, 2);
  Serial.print(",");
  Serial.print(nsScrFrequencyPerMin, 2);
  Serial.print(",");
  Serial.print(baselinePhasicMean, 2);
  Serial.print(",");
  Serial.print(baselinePhasicStd, 2);
  Serial.print(",");
  Serial.print(featureReady);
  Serial.print(",");
  Serial.print(MIN_SCR_THRESHOLD_ADC, 2);
  Serial.print(",");
  Serial.println(MAX_SCR_THRESHOLD_ADC, 2);
}


// =====================
// Setup
// =====================
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < MA_N; i++) {
    maBuf[i] = 0.0;
  }

  printHeader();
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

  // ---------------------
  // 샘플 시간 업데이트
  // ---------------------
  prev2SampleTimeMs = prev1SampleTimeMs;
  prev1SampleTimeMs = currSampleTimeMs;
  currSampleTimeMs = now;

  // ---------------------
  // 1. Data acquisition
  // ---------------------
  int rawGsr = analogRead(GSR_PIN);
  float rawGsrFloat = (float)rawGsr;

  // ---------------------
  // 2. Median filter N=3
  // ---------------------
  float medianGsr = updateMedianFilter(rawGsrFloat);

  // ---------------------
  // 3a. SCL용 Moving average N=150
  // SCL Mean feature 계산에 사용
  // ---------------------
  float sclBaseline = updateMovingAverage(medianGsr);

  // ---------------------
  // 3b. SCR 검출용 EMA baseline
  // α=0.02, τ≈5초
  // phasic 계산 및 SCR peak 검출에 사용
  // ---------------------
  float scrDetectBl = updateEmaBaseline(medianGsr);

  // ---------------------
  // 4. Phasic separation
  // SCR 검출용 EMA baseline 기준
  // ---------------------
  float phasic = medianGsr - scrDetectBl;

  float phasicPos = phasic;
  if (phasicPos < 0.0) {
    phasicPos = 0.0;
  }

  // ---------------------
  // Mode definition
  // 0: warm-up
  // 1: baseline
  // 2: measurement
  // ---------------------
  int mode = 0;

  if (sampleIndex < WARMUP_SAMPLES) {
    mode = 0;
  } else if (sampleIndex < BASELINE_END_SAMPLES) {
    mode = 1;
  } else {
    mode = 2;
  }

  // ---------------------
  // 5. Baseline statistics
  // EMA 기반 unclipped phasic 기준
  // ---------------------
  if (mode == 1) {
    baselinePhasicSum += phasic;
    baselinePhasicSqSum += phasic * phasic;
    baselineCount++;
  }

  // 60초 이후 measurement 진입 시 baseline 확정
  if (!baselineReady && sampleIndex == BASELINE_END_SAMPLES) {
    finalizeBaseline();
  }

  // ---------------------
  // 6. SCR peak detection
  // 현재 샘플이 들어온 뒤,
  // 직전 샘플 phasicPrev1이 국소 최대점인지 판단
  // ---------------------
  int scrFlag = 0;
  detectedScrSampleIndex = -1;
  detectedScrTimeMs = 0;

  phasicPrev2 = phasicPrev1;
  phasicPrev1 = phasicCurr;
  phasicCurr = phasicPos;

  if (mode == 2 && baselineReady) {
    bool localPeak =
      (phasicPrev1 > phasicPrev2) &&
      (phasicPrev1 >= phasicCurr);

    bool aboveThreshold =
      (phasicPrev1 >= scrThreshold) &&
      (phasicPrev1 > 0.0);

    bool refractoryOk =
      ((sampleIndex - 1) - lastScrSampleIndex) >= REFRACTORY_SAMPLES;

    if (localPeak && aboveThreshold && refractoryOk) {
      scrFlag = 1;

      // 실제 피크는 현재 샘플이 아니라 직전 샘플
      detectedScrSampleIndex = sampleIndex - 1;
      detectedScrTimeMs = prev1SampleTimeMs;

      lastScrSampleIndex = detectedScrSampleIndex;

      windowScrCount++;
      windowScrAmpSum += phasicPrev1;
    }
  }

  // ---------------------
  // 7. Feature window
  // Measurement 구간에서만 30초 단위 특징값 계산
  // SCL Mean은 sclBaseline (15초 MA) 기준
  // ---------------------
  float sclMean30s = 0.0;
  float scrAmplitudeMean30s = 0.0;
  float nsScrFrequencyPerMin = 0.0;
  int featureReady = 0;

  if (mode == 2 && baselineReady) {
    windowSclSum += sclBaseline;
    windowSampleCount++;

    if (windowSampleCount >= FEATURE_WINDOW_SAMPLES) {
      featureReady = 1;

      // SCL Mean (15초 MA 기반)
      sclMean30s = windowSclSum / windowSampleCount;

      // SCR Amplitude Mean
      if (windowScrCount > 0) {
        scrAmplitudeMean30s = windowScrAmpSum / windowScrCount;
      } else {
        scrAmplitudeMean30s = 0.0;
      }

      // NS-SCR Frequency
      // 30초 동안 검출된 SCR 개수를 1분 단위로 환산
      nsScrFrequencyPerMin =
        windowScrCount * (60.0 / FEATURE_WINDOW_SEC);

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
        featureReady
      );

      // 다음 30초 feature window 초기화
      windowSclSum = 0.0;
      windowSampleCount = 0;
      windowScrAmpSum = 0.0;
      windowScrCount = 0;

      sampleIndex++;
      return;
    }
  }

  // ---------------------
  // 출력 정책
  // ---------------------
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
      featureReady
    );
  }

  sampleIndex++;
}
