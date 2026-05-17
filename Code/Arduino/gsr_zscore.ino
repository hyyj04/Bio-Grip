/*
  GSR Signal Processing for Arduino IDE

  Version:
  Dual Baseline + Contact Check + Z-score + Stress Score

  Pipeline:
  1. Data Acquisition: 10 Hz
  2. Contact validity check
  3. Median Filter, N = 3
  4. SCL baseline: Moving Average, N = 150
  5. SCR detection baseline: EMA
  6. Phasic = medianGSR - scrDetectBaseline
  7. Baseline: 15~60 sec
  8. Measurement: after 60 sec
  9. 30 sec feature extraction
  10. Baseline-based z-score
  11. GSR stress score
*/

#define GSR_PIN A0

// =====================
// CSV output mode
// =====================
const bool PRINT_EVERY_SAMPLE = true;

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
const int BASELINE_END_SEC = 60;
const int FEATURE_WINDOW_SEC = 30;

const int MA_FILL_SAMPLES = MA_N;
const int USER_WARMUP_SAMPLES = USER_WARMUP_SEC * FS_GSR;

const int WARMUP_SAMPLES =
  (USER_WARMUP_SAMPLES > MA_FILL_SAMPLES) ? USER_WARMUP_SAMPLES : MA_FILL_SAMPLES;

const int BASELINE_END_SAMPLES = BASELINE_END_SEC * FS_GSR;
const int FEATURE_WINDOW_SAMPLES = FEATURE_WINDOW_SEC * FS_GSR;

// =====================
// SCR detection parameters
// =====================
const float THRESHOLD_K = 2.0;
const float MIN_SCR_THRESHOLD_ADC = 6.0;
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
  Serial.print(baselineSclMean, 2);
  Serial.print(",");
  Serial.print(baselineSclStd, 2);
  Serial.print(",");
  Serial.print(baselinePhasicMean, 2);
  Serial.print(",");
  Serial.print(baselinePhasicStd, 2);
  Serial.print(",");
  Serial.print(baselinePhasicPosMean, 2);
  Serial.print(",");
  Serial.print(baselinePhasicPosStd, 2);
  Serial.print(",");
  Serial.print(sclMeanZ, 2);
  Serial.print(",");
  Serial.print(scrAmplitudeMeanZ, 2);
  Serial.print(",");
  Serial.print(nsScrFrequencyZ, 2);
  Serial.print(",");
  Serial.print(sclStressScore, 2);
  Serial.print(",");
  Serial.print(scrAmplitudeStressScore, 2);
  Serial.print(",");
  Serial.print(nsScrFrequencyStressScore, 2);
  Serial.print(",");
  Serial.print(finalGsrStressScore, 2);
  Serial.print(",");
  Serial.print(featureReady);
  Serial.print(",");
  Serial.print(MIN_SCR_THRESHOLD_ADC, 2);
  Serial.print(",");
  Serial.print(MAX_SCR_THRESHOLD_ADC, 2);
  Serial.print(",");
  Serial.print(contactOk);
  Serial.print(",");
  Serial.println(validSampleFlag);
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

  prev2SampleTimeMs = prev1SampleTimeMs;
  prev1SampleTimeMs = currSampleTimeMs;
  currSampleTimeMs = now;

  int mode = 0;

  if (sampleIndex < WARMUP_SAMPLES) {
    mode = 0;
  } else if (sampleIndex < BASELINE_END_SAMPLES) {
    mode = 1;
  } else {
    mode = 2;
  }

  int rawGsr = analogRead(GSR_PIN);

  bool contactOkBool = rawGsr >= MIN_VALID_GSR_ADC;
  int contactOk = contactOkBool ? 1 : 0;

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

  // Baseline statistics
  if (mode == 1 && validSampleFlag == 1) {
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
  if (mode == 2 && baselineReady && validSampleFlag == 1) {
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

    if (localPeak && aboveThreshold && refractoryOk) {
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
  if (mode == 2 && baselineReady && validSampleFlag == 1) {
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

      finalGsrStressScore =
        (sclStressScore +
         scrAmplitudeStressScore +
         nsScrFrequencyStressScore) / 3.0;

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