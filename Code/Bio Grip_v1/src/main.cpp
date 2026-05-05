// ================================================================
// stress_monitor.ino
// PPG (MAX30102) + GSR 기반 스트레스 측정 통합 코드
//
// 전체 파이프라인:
//   GSR  : 아날로그 읽기 → 접촉 확인 → 미디언 필터 → SCL/SCR 베이스라인
//          → Phasic 계산 → SCR 피크 검출 → 30초 피처 추출 → Z-score → 스트레스 점수
//   PPG  : IR 읽기 → 밴드패스 필터 → 피크 검출 → IBI → HRV → Z-score → 스트레스 점수
//   통합 : PPG 점수 + GSR 점수 → 조건부 평균 → 최종 통합 스트레스 점수
// ================================================================

#include <Wire.h>
#include "MAX30105.h"

MAX30105 particleSensor;


// ================================================================
// [GSR 모듈]
// 피부 전기 반응(GSR) 신호를 처리하고 스트레스 점수를 계산한다.
// namespace로 감싸서 PPG 변수와의 이름 충돌을 방지한다.
// ================================================================
namespace gsr {

// ----------------------------------------------------------------
// [GSR] 핀 설정
// ----------------------------------------------------------------
#define GSR_PIN A0  // GSR 센서 아날로그 입력 핀

// ----------------------------------------------------------------
// [GSR] 디버그 옵션
// ----------------------------------------------------------------
// true  : 매 샘플(100ms)마다 CSV 출력 → 신호 파형 디버깅용
// false : 30초 피처 완성 시에만 출력  → 실제 운용 모드
const bool PRINT_EVERY_SAMPLE = false;

// ----------------------------------------------------------------
// [GSR] 샘플링 설정
// ----------------------------------------------------------------
const unsigned long SAMPLE_INTERVAL_MS = 100;  // 샘플 주기 (ms)마다 1번 샘플링
const int           FS_GSR             = 10;   // 샘플링 주파수 (Hz) = 1000 / 100

// ----------------------------------------------------------------
// [GSR] 접촉 유효성 설정
// ----------------------------------------------------------------
const int MIN_VALID_GSR_ADC      = 10;  // ADC된 값이 이 미만이면 센서 미접촉으로 판단
const int CONTACT_RECOVERY_SAMPLES = 10; // 접촉 복구 직후 신뢰하지 않을 샘플 수
int contactRecoveryCount = 0;            // 현재 복구 대기 카운트

// ----------------------------------------------------------------
// [GSR] 필터 설정
// ----------------------------------------------------------------
const int   MEDIAN_N           = 3;     // 미디언 필터 윈도우 크기 (샘플 수)
const int   MA_N               = 150;   // 이동평균 윈도우 크기 (15초 × 10Hz = 150샘플)
const float SCR_BASELINE_ALPHA = 0.02;  // EMA 평활 계수: 새 값에 2%만 반응

// ----------------------------------------------------------------
// [GSR] 측정 구간 시간 설정
// ----------------------------------------------------------------
const int USER_WARMUP_SEC      = 15;    // 워밍업 구간 (초)
const int BASELINE_DURATION_SEC = 60;  // 베이스라인 측정 구간 (초)
const int BASELINE_END_SEC     = USER_WARMUP_SEC + BASELINE_DURATION_SEC; // 75초
const int FEATURE_WINDOW_SEC   = 30;   // 피처 추출 윈도우 크기 (초)

// 샘플 수 단위로 변환(초 --> 샘플 수)
const int MA_FILL_SAMPLES       = MA_N; // 이동 평균 버퍼가 꽉 차기 위한 샘플 수 (15s = 150samples)
const int USER_WARMUP_SAMPLES   = USER_WARMUP_SEC * FS_GSR; // 워밍업 시간을 샘플 수로 변환 (15초 x 10Hz = 150sample)
const int WARMUP_SAMPLES        = (USER_WARMUP_SAMPLES > MA_FILL_SAMPLES)
                                    ? USER_WARMUP_SAMPLES : MA_FILL_SAMPLES; // 워밍업 샘플 & MA 버퍼 샘플이 모두 찼는지 확인
const int BASELINE_END_SAMPLES  = BASELINE_END_SEC * FS_GSR;             // 750 샘플 (75초)
const int REST_END_SAMPLES      = BASELINE_END_SAMPLES + 300 * FS_GSR;   // 3750 샘플 (375초)
const int TRANSITION_END_SAMPLES= REST_END_SAMPLES    + 30  * FS_GSR;    // 4050 샘플 (405초)
const int STRESS_END_SAMPLES    = TRANSITION_END_SAMPLES + 270 * FS_GSR; // 6750 샘플 (675초)
const int FEATURE_WINDOW_SAMPLES= FEATURE_WINDOW_SEC * FS_GSR;           // 300 샘플 (30초), 300샘플마다 스트레스 점수를 계산

// ----------------------------------------------------------------
// [GSR] SCR 검출 설정
// ----------------------------------------------------------------
const float THRESHOLD_K          = 2.5;   // 임계값 = 베이스라인 표준편차 × 이 값
const float MIN_SCR_THRESHOLD_ADC = 6.0;  // 임계값 하한 (너무 민감해지는 것 방지)
const float MAX_SCR_THRESHOLD_ADC = 15.0; // 임계값 상한 (너무 둔해지는 것 방지)
const float MIN_SCR_RISE_ADC     = 2.0;   // 이 미만의 상승은 잡음으로 간주
const int   REFRACTORY_SAMPLES   = 10;    // SCR 검출 후 재검출 금지 구간 (샘플 수)

// ----------------------------------------------------------------
// [GSR] Z-score 및 스트레스 점수 설정
// ----------------------------------------------------------------
// 표준편차가 너무 작을 때 Z-score 폭주 방지를 위한 최소값
const float MIN_SCL_STD_FOR_Z    = 1.0;
const float MIN_PHASIC_STD_FOR_Z = 1.0;
const float MIN_FREQ_STD_FOR_Z   = 2.0;

// Z-score → 점수 변환 스케일
// z = 0 → 50점 / z = +1 → 65점 / z = +2 → 80점 / z = +3 → 95점
const float Z_SCORE_SCALE = 15.0;

// ----------------------------------------------------------------
// [GSR] 필터 버퍼 변수
// ----------------------------------------------------------------
// 미디언 필터
float medBuf[MEDIAN_N] = {0, 0, 0}; // 최근 3개 샘플 원형 버퍼
int   medIndex = 0;                  // 다음 쓰기 위치
bool  medFilled = false;             // 버퍼가 한 번이라도 꽉 찼는지 여부

// SCL 이동평균 필터
float maBuf[MA_N];   // 최근 150개 샘플 원형 버퍼
int   maIndex = 0;   // 다음 쓰기 위치
int   maCount = 0;   // 현재 버퍼에 쌓인 샘플 수
float maSum   = 0.0; // 합산값 (평균 계산용)

// SCR EMA 베이스라인
float scrDetectBaseline      = 0.0;   // 현재 EMA 값
bool  scrBaselineInitialized = false; // 첫 샘플로 초기화됐는지 여부

// ----------------------------------------------------------------
// [GSR] 최근 유효 신호 보존용 변수
// 접촉 불량 구간에서도 마지막으로 계산된 값을 유지한다.
// ----------------------------------------------------------------
float lastMedianGsr         = 0.0;
float lastSclBaseline       = 0.0;
float lastScrDetectBaseline = 0.0;
bool  hasValidSignal        = false;

// ----------------------------------------------------------------
// [GSR] 베이스라인 통계 누적 변수
// CALIBRATION 구간 동안 합계/제곱합을 누적해 평균·표준편차를 계산한다.
// ----------------------------------------------------------------
// SCR 임계값 산출용 (phasic 기준)
float baselinePhasicSum   = 0.0;
float baselinePhasicSqSum = 0.0;
int   baselineCount       = 0;
float baselinePhasicMean  = 0.0;
float baselinePhasicStd   = 0.0;

// Z-score 산출용: SCL
float baselineSclSum    = 0.0;
float baselineSclSqSum  = 0.0;
int   baselineSclCount  = 0;
float baselineSclMean   = 0.0;
float baselineSclStd    = 0.0;

// Z-score 산출용: Phasic Pos (SCR 진폭)
float baselinePhasicPosSum   = 0.0;
float baselinePhasicPosSqSum = 0.0;
int   baselinePhasicPosCount = 0;
float baselinePhasicPosMean  = 0.0;
float baselinePhasicPosStd   = 0.0;

// NS-SCR 빈도: 안정 상태에서 0회/분으로 가정
float baselineNsScrFreqMean = 0.0;
float baselineNsScrFreqStd  = MIN_FREQ_STD_FOR_Z;

// ----------------------------------------------------------------
// [GSR] SCR 임계값 및 베이스라인 준비 플래그
// ----------------------------------------------------------------
float scrThreshold  = MIN_SCR_THRESHOLD_ADC; // finalizeBaseline() 후 갱신
bool  baselineReady = false;

// ----------------------------------------------------------------
// [GSR] 30초 피처 윈도우 누적 변수
// ----------------------------------------------------------------
float windowSclSum      = 0.0;
int   windowSampleCount = 0;
float windowScrAmpSum   = 0.0;
int   windowScrCount    = 0;

// ----------------------------------------------------------------
// [GSR] SCR 피크 검출용 3-샘플 슬라이딩 윈도우
// prev2 → prev1 → curr 순서로 밀려난다.
// prev1이 봉우리인지 curr을 받은 후에 확인할 수 있다.
// ----------------------------------------------------------------
float phasicPrev2 = 0.0;
float phasicPrev1 = 0.0;
float phasicCurr  = 0.0;

int           lastScrSampleIndex   = -9999; // 마지막 SCR 검출 샘플 인덱스
int           detectedScrSampleIndex = -1;
unsigned long detectedScrTimeMs      = 0;

// ----------------------------------------------------------------
// [GSR] 샘플 타임스탬프 (3-샘플 슬라이딩 윈도우에 대응)
// ----------------------------------------------------------------
unsigned long prev2SampleTimeMs = 0;
unsigned long prev1SampleTimeMs = 0;
unsigned long currSampleTimeMs  = 0;

// ----------------------------------------------------------------
// [GSR] 일반 변수
// ----------------------------------------------------------------
unsigned long lastSampleTime = 0;
int           sampleIndex    = 0;

// ----------------------------------------------------------------
// [GSR] 통합 출력용 최신 GSR 값
// 30초 윈도우 단위로 갱신되므로 freshness time으로 유효성을 관리한다.
// ----------------------------------------------------------------
String latestMode            = "STABILIZING";
int    latestRawGsr          = 0;
int    latestSampleIndex     = 0;
int    latestContactOk       = 0;
int    latestValidSampleFlag = 0;

float latestMedianGsr            = NAN;
float latestSclBaseline          = NAN;
float latestScrDetectBaseline    = NAN;
float latestPhasic               = NAN;
float latestPhasicPos            = NAN;
float latestSclMean30s           = NAN;
float latestScrAmplitudeMean30s  = NAN;
float latestNsScrFrequencyPerMin = NAN;
float latestSclMeanZ             = NAN;
float latestScrAmplitudeMeanZ    = NAN;
float latestNsScrFrequencyZ      = NAN;
float latestSclStressScore           = NAN;
float latestScrAmplitudeStressScore  = NAN;
float latestNsScrFrequencyStressScore= NAN;
float latestFinalGsrStressScore      = NAN;
int   latestFeatureReady             = 0;

// 30초 윈도우 + 여유 10초 → 40초 이내의 GSR 점수만 유효로 취급
const unsigned long GSR_SCORE_FRESHNESS_MS = (FEATURE_WINDOW_SEC + 10UL) * 1000UL;
unsigned long latestGsrScoreUpdateMs = 0;

bool isLatestScoreFresh(unsigned long nowMs) {
  if (latestGsrScoreUpdateMs == 0) return false;
  return (nowMs - latestGsrScoreUpdateMs) <= GSR_SCORE_FRESHNESS_MS;
}


// ================================================================
// [GSR] 유틸리티 함수
// ================================================================

// 세 값 중 중간값 반환 (순간 잡음 제거용)
float median3(float a, float b, float c) {
  if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
  if ((b >= a && b <= c) || (b <= a && b >= c)) return b;
  return c;
}

// 값을 [low, high] 범위로 클램프
float clampFloat(float x, float low, float high) {
  if (x < low)  return low;
  if (x > high) return high;
  return x;
}

// 표준편차가 minStd보다 작으면 minStd 반환 (Z-score 폭주 방지)
float safeStd(float stdVal, float minStd) {
  return (stdVal < minStd) ? minStd : stdVal;
}

// Z = (value - mean) / std (최소 표준편차 적용)
float computeZScore(float value, float meanVal, float stdVal, float minStd) {
  return (value - meanVal) / safeStd(stdVal, minStd);
}

// Z-score → 0~100 스트레스 점수 변환
// z = 0 → 50점, z = ±3 → 95점 / 5점
float zToStressScore(float z) {
  float zClamped = clampFloat(z, -3.0, 3.0);
  return clampFloat(50.0 + zClamped * Z_SCORE_SCALE, 0.0, 100.0);
}

// 합계/개수로 평균 계산
float calcMean(float sumVal, int countVal) {
  if (countVal <= 0) return 0.0;
  return sumVal / countVal;
}

// 합계/제곱합/개수로 표준편차 계산 (전체 배열 없이 온라인 계산)
float calcStd(float sumVal, float sqSumVal, int countVal) {
  if (countVal <= 1) return 0.0;
  float meanVal  = sumVal / countVal;
  float variance = (sqSumVal / countVal) - (meanVal * meanVal);
  if (variance < 0.0) variance = 0.0;
  return sqrt(variance);
}


// ================================================================
// [GSR] 신호 필터 함수
// ================================================================

// 미디언 필터: 원형 버퍼에 새 샘플을 추가하고 중간값 반환
float updateMedianFilter(float x) {
  medBuf[medIndex] = x;
  if (++medIndex >= MEDIAN_N) {
    medIndex = 0;
    medFilled = true;
  }
  return medFilled ? median3(medBuf[0], medBuf[1], medBuf[2]) : x;
}

// 이동평균 필터: 원형 버퍼로 가장 오래된 값을 빼고 새 값을 더해 효율적으로 갱신
float updateMovingAverage(float x) {
  if (maCount < MA_N) {
    // 버퍼가 아직 안 찬 경우: 그냥 추가
    maBuf[maIndex] = x;
    maSum += x;
    maCount++;
  } else {
    // 버퍼가 꽉 찬 경우: 가장 오래된 값을 빼고 새 값을 더함
    maSum -= maBuf[maIndex];
    maBuf[maIndex] = x;
    maSum += x;
  }
  if (++maIndex >= MA_N) maIndex = 0;
  return maSum / maCount;
}

// EMA(지수이동평균) 베이스라인: 첫 샘플로 초기화, 이후 α=0.02로 느리게 추적
// α가 작을수록 천천히 반응 → SCR(빠른 성분) 제거 후 SCL(느린 성분) 추적
float updateEmaBaseline(float x) {
  if (!scrBaselineInitialized) {
    scrDetectBaseline = x;
    scrBaselineInitialized = true;
  } else {
    scrDetectBaseline = SCR_BASELINE_ALPHA * x
                      + (1.0 - SCR_BASELINE_ALPHA) * scrDetectBaseline;
  }
  return scrDetectBaseline;
}


// ================================================================
// [GSR] 베이스라인 확정 함수
// CALIBRATION 구간 종료 시 한 번 호출되어 통계를 확정한다.
// ================================================================
void finalizeBaseline() {
  // 평균·표준편차 확정
  baselinePhasicMean   = calcMean(baselinePhasicSum,   baselineCount);
  baselinePhasicStd    = calcStd (baselinePhasicSum,   baselinePhasicSqSum,   baselineCount);
  baselineSclMean      = calcMean(baselineSclSum,      baselineSclCount);
  baselineSclStd       = calcStd (baselineSclSum,      baselineSclSqSum,      baselineSclCount);
  baselinePhasicPosMean= calcMean(baselinePhasicPosSum, baselinePhasicPosCount);
  baselinePhasicPosStd = calcStd (baselinePhasicPosSum, baselinePhasicPosSqSum, baselinePhasicPosCount);

  // SCR 검출 임계값 확정: baseline std × 2.5, 단 [MIN, MAX] 범위로 클램프
  scrThreshold = clampFloat(
    THRESHOLD_K * baselinePhasicStd,
    MIN_SCR_THRESHOLD_ADC,
    MAX_SCR_THRESHOLD_ADC
  );

  // NS-SCR 빈도 베이스라인: 안정 구간에서는 0회/분으로 가정
  baselineNsScrFreqMean = 0.0;
  baselineNsScrFreqStd  = MIN_FREQ_STD_FOR_Z;

  baselineReady = true;
}

// SCR 피크 검출 슬라이딩 버퍼 초기화
void resetPeakDetectionBuffer() {
  phasicPrev2 = phasicPrev1 = phasicCurr = 0.0;
  lastScrSampleIndex    = sampleIndex;
  detectedScrSampleIndex = -1;
  detectedScrTimeMs      = 0;
}


// ================================================================
// [GSR] CSV 출력 함수
// ================================================================
void printHeader() {
  Serial.println(
    "time_ms,sample_index,mode,raw_gsr,median_gsr,"
    "scl_baseline,scr_detect_baseline,phasic,phasic_pos,"
    "scr_threshold,scr_flag,scr_peak_time_ms,scr_peak_sample_index,"
    "scl_mean_30s,scr_amplitude_mean_30s,ns_scr_frequency_per_min,"
    "baseline_scl_mean,baseline_scl_std,baseline_phasic_mean,baseline_phasic_std,"
    "baseline_phasic_pos_mean,baseline_phasic_pos_std,"
    "scl_mean_z,scr_amplitude_mean_z,ns_scr_frequency_z,"
    "scl_stress_score,scr_amplitude_stress_score,ns_scr_frequency_stress_score,"
    "final_gsr_stress_score,feature_ready,"
    "min_scr_threshold_adc,max_scr_threshold_adc,contact_ok,valid_sample_flag"
  );
}

// 통합 버전에서는 PPG loop가 통합 CSV를 출력하므로 이 함수는 비워둔다.
void printCsvRow(
  unsigned long timeMs, int sIndex, String mode, int rawGsr,
  float medianGsr, float sclBaseline, float scrDetectBl,
  float phasic, float phasicPos,
  int scrFlag, unsigned long scrPeakTimeMs, int scrPeakSampleIndex,
  float sclMean30s, float scrAmplitudeMean30s, float nsScrFrequencyPerMin,
  float sclMeanZ, float scrAmplitudeMeanZ, float nsScrFrequencyZ,
  float sclStressScore, float scrAmplitudeStressScore, float nsScrFrequencyStressScore,
  float finalGsrStressScore, int featureReady, int contactOk, int validSampleFlag
) {
  // Suppressed: unified CSV is printed by the main PPG loop.
}


// ================================================================
// [GSR] 초기화 함수
// ================================================================
void setup() {
  for (int i = 0; i < MA_N; i++) maBuf[i] = 0.0;
}

void resetAll() {
  // 미디언 필터
  for (int i = 0; i < MEDIAN_N; i++) medBuf[i] = 0.0;
  medIndex = 0; medFilled = false;

  // 이동평균 필터
  for (int i = 0; i < MA_N; i++) maBuf[i] = 0.0;
  maIndex = 0; maCount = 0; maSum = 0.0;

  // EMA 베이스라인
  scrDetectBaseline = 0.0; scrBaselineInitialized = false;

  // 최근 유효 신호
  lastMedianGsr = lastSclBaseline = lastScrDetectBaseline = 0.0;
  hasValidSignal = false;

  // 베이스라인 통계
  baselinePhasicSum = baselinePhasicSqSum = 0.0; baselineCount = 0;
  baselinePhasicMean = baselinePhasicStd = 0.0;

  baselineSclSum = baselineSclSqSum = 0.0; baselineSclCount = 0;
  baselineSclMean = baselineSclStd = 0.0;

  baselinePhasicPosSum = baselinePhasicPosSqSum = 0.0; baselinePhasicPosCount = 0;
  baselinePhasicPosMean = baselinePhasicPosStd = 0.0;

  baselineNsScrFreqMean = 0.0; baselineNsScrFreqStd = MIN_FREQ_STD_FOR_Z;

  // 임계값 및 플래그
  scrThreshold = MIN_SCR_THRESHOLD_ADC; baselineReady = false;

  // 피처 윈도우
  windowSclSum = 0.0; windowSampleCount = 0;
  windowScrAmpSum = 0.0; windowScrCount = 0;

  // SCR 피크 검출
  phasicPrev2 = phasicPrev1 = phasicCurr = 0.0;
  lastScrSampleIndex = -9999;
  detectedScrSampleIndex = -1; detectedScrTimeMs = 0;

  // 타임스탬프
  prev2SampleTimeMs = prev1SampleTimeMs = currSampleTimeMs = 0;

  // 일반
  sampleIndex = 0; contactRecoveryCount = 0;

  // 최신 출력값
  latestMode = "STABILIZING";
  latestRawGsr = latestSampleIndex = latestContactOk = latestValidSampleFlag = 0;
  latestMedianGsr = latestSclBaseline = latestScrDetectBaseline = NAN;
  latestPhasic = latestPhasicPos = NAN;
  latestSclMean30s = latestScrAmplitudeMean30s = latestNsScrFrequencyPerMin = NAN;
  latestSclMeanZ = latestScrAmplitudeMeanZ = latestNsScrFrequencyZ = NAN;
  latestSclStressScore = latestScrAmplitudeStressScore = latestNsScrFrequencyStressScore = NAN;
  latestFinalGsrStressScore = NAN;
  latestFeatureReady = 0; latestGsrScoreUpdateMs = 0;
}


// ================================================================
// [GSR] 메인 루프
// PPG loop()에서 매 사이클마다 호출된다.
// ================================================================
void loop() {
  unsigned long now = millis();

  // 100ms 간격(10Hz)으로만 실행
  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) return;
  lastSampleTime = now;

  // 3-샘플 타임스탬프 슬라이딩 업데이트
  prev2SampleTimeMs = prev1SampleTimeMs;
  prev1SampleTimeMs = currSampleTimeMs;
  currSampleTimeMs  = now;

  // ── 모드 결정 ──────────────────────────────────────────────
  String mode;
  if      (sampleIndex < WARMUP_SAMPLES)         mode = "STABILIZING";
  else if (sampleIndex < BASELINE_END_SAMPLES)   mode = "CALIBRATION";
  else if (sampleIndex < REST_END_SAMPLES)       mode = "REST";
  else if (sampleIndex < TRANSITION_END_SAMPLES) mode = "TRANSITION";
  else if (sampleIndex < STRESS_END_SAMPLES)     mode = "STRESS";
  else                                            mode = "DONE";

  // ── ADC 읽기 및 접촉 확인 ──────────────────────────────────
  int  rawGsr      = analogRead(GSR_PIN);
  bool contactOkBool = (rawGsr >= MIN_VALID_GSR_ADC);
  int  contactOk   = contactOkBool ? 1 : 0;

  latestMode        = mode;
  latestRawGsr      = rawGsr;
  latestSampleIndex = sampleIndex;
  latestContactOk   = contactOk;

  // 로컬 변수 초기화 (매 샘플마다 리셋)
  float medianGsr  = lastMedianGsr;
  float sclBaseline= lastSclBaseline;
  float scrDetectBl= lastScrDetectBaseline;
  float phasic = 0.0, phasicPos = 0.0;
  int   scrFlag = 0;
  detectedScrSampleIndex = -1; detectedScrTimeMs = 0;

  float sclMean30s = 0.0, scrAmplitudeMean30s = 0.0, nsScrFrequencyPerMin = 0.0;
  float sclMeanZ = 0.0,   scrAmplitudeMeanZ = 0.0,   nsScrFrequencyZ = 0.0;
  float sclStressScore = 0.0, scrAmplitudeStressScore = 0.0;
  float nsScrFrequencyStressScore = 0.0, finalGsrStressScore = 0.0;
  int   featureReady = 0;

  // ── 베이스라인 종료 처리 ────────────────────────────────────
  // CALIBRATION이 끝났는데 아직 확정이 안 됐으면 강제 확정
  if (!baselineReady && sampleIndex >= BASELINE_END_SAMPLES) {
    finalizeBaseline();
    resetPeakDetectionBuffer();
  }

  // ── 접촉 불량 처리 ──────────────────────────────────────────
  if (!contactOkBool) {
    contactRecoveryCount = CONTACT_RECOVERY_SAMPLES;

    if (!hasValidSignal) {
      medianGsr = sclBaseline = scrDetectBl = 0.0;
    }

    latestMedianGsr       = medianGsr;
    latestSclBaseline     = sclBaseline;
    latestScrDetectBaseline = scrDetectBl;
    latestPhasic          = phasic;
    latestPhasicPos       = phasicPos;
    latestValidSampleFlag = 0;
    latestFeatureReady    = 0;
    // GSR 점수는 즉시 지우지 않음 → 통합 단계에서 freshness로 판단

    if (PRINT_EVERY_SAMPLE) {
      printCsvRow(now, sampleIndex, mode, rawGsr,
        medianGsr, sclBaseline, scrDetectBl, phasic, phasicPos,
        scrFlag, detectedScrTimeMs, detectedScrSampleIndex,
        sclMean30s, scrAmplitudeMean30s, nsScrFrequencyPerMin,
        sclMeanZ, scrAmplitudeMeanZ, nsScrFrequencyZ,
        sclStressScore, scrAmplitudeStressScore, nsScrFrequencyStressScore,
        finalGsrStressScore, featureReady, contactOk, 0);
    }

    sampleIndex++;
    return;
  }

  // ── 신호 필터링 ─────────────────────────────────────────────
  medianGsr  = updateMedianFilter((float)rawGsr); // 순간 잡음 제거
  sclBaseline= updateMovingAverage(medianGsr);    // 천천히 변하는 SCL 추적
  scrDetectBl= updateEmaBaseline(medianGsr);      // 빠르게 변하는 SCR 기준선

  lastMedianGsr         = medianGsr;
  lastSclBaseline       = sclBaseline;
  lastScrDetectBaseline = scrDetectBl;
  hasValidSignal        = true;

  // 접촉 복구 직후 샘플은 유효하지 않은 것으로 표시
  int validSampleFlag = 1;
  if (contactRecoveryCount > 0) {
    contactRecoveryCount--;
    validSampleFlag = 0;
  }

  // ── Phasic 신호 계산 ────────────────────────────────────────
  // phasic     = SCR 성분 (음수 포함)
  // phasicPos  = SCR 성분 중 양의 부분만 (SCR 진폭 계산에 사용)
  phasic    = medianGsr - scrDetectBl;
  phasicPos = max(phasic, 0.0f);

  latestMedianGsr         = medianGsr;
  latestSclBaseline       = sclBaseline;
  latestScrDetectBaseline = scrDetectBl;
  latestPhasic            = phasic;
  latestPhasicPos         = phasicPos;
  latestValidSampleFlag   = validSampleFlag;

  // 새 30초 윈도우가 완성되기 전이라도, 최근 점수가 신선하면 유효로 표시
  latestFeatureReady = isLatestScoreFresh(now) ? 1 : 0;

  // ── CALIBRATION: 베이스라인 통계 누적 ───────────────────────
  if (mode == "CALIBRATION" && validSampleFlag == 1) {
    baselinePhasicSum   += phasic;
    baselinePhasicSqSum += phasic * phasic;
    baselineCount++;

    baselineSclSum    += sclBaseline;
    baselineSclSqSum  += sclBaseline * sclBaseline;
    baselineSclCount++;

    baselinePhasicPosSum   += phasicPos;
    baselinePhasicPosSqSum += phasicPos * phasicPos;
    baselinePhasicPosCount++;
  }

  // ── SCR 피크 검출 ────────────────────────────────────────────
  // REST/STRESS 구간에서만 수행, 베이스라인 확정 후 유효 샘플에 대해서만 실행
  if ((mode == "REST" || mode == "STRESS") && baselineReady && validSampleFlag == 1) {
    // 슬라이딩 윈도우 밀기
    phasicPrev2 = phasicPrev1;
    phasicPrev1 = phasicCurr;
    phasicCurr  = phasicPos;

    // 4가지 조건이 모두 참이어야 SCR로 인정
    bool localPeak      = (phasicPrev1 > phasicPrev2) && (phasicPrev1 >= phasicCurr);
    bool aboveThreshold = (phasicPrev1 >= scrThreshold) && (phasicPrev1 > 0.0);
    bool refractoryOk   = ((sampleIndex - 1) - lastScrSampleIndex) >= REFRACTORY_SAMPLES;
    bool riseOk         = (phasicPrev1 - phasicPrev2) >= MIN_SCR_RISE_ADC;

    if (localPeak && aboveThreshold && refractoryOk && riseOk) {
      scrFlag = 1;
      detectedScrSampleIndex = sampleIndex - 1; // 봉우리는 prev1 → 한 샘플 전
      detectedScrTimeMs      = prev1SampleTimeMs;
      lastScrSampleIndex     = detectedScrSampleIndex;

      windowScrCount++;
      windowScrAmpSum += phasicPrev1;
    }
  } else {
    // REST/STRESS 밖에서는 슬라이딩 버퍼 초기화
    phasicPrev2 = phasicPrev1 = phasicCurr = 0.0;
  }

  // ── 30초 피처 윈도우 및 스트레스 점수 계산 ──────────────────
  if ((mode == "REST" || mode == "STRESS") && baselineReady && validSampleFlag == 1) {
    windowSclSum += sclBaseline;
    windowSampleCount++;

    // 300샘플(30초)이 채워지면 피처 계산
    if (windowSampleCount >= FEATURE_WINDOW_SAMPLES) {
      featureReady = 1;

      // 피처 계산
      sclMean30s = windowSclSum / windowSampleCount;
      scrAmplitudeMean30s = (windowScrCount > 0)
        ? windowScrAmpSum / windowScrCount : 0.0;
      nsScrFrequencyPerMin = windowScrCount * (60.0 / FEATURE_WINDOW_SEC);

      // Z-score 계산
      sclMeanZ         = computeZScore(sclMean30s,          baselineSclMean,      baselineSclStd,      MIN_SCL_STD_FOR_Z);
      scrAmplitudeMeanZ= computeZScore(scrAmplitudeMean30s, baselinePhasicPosMean, baselinePhasicPosStd, MIN_PHASIC_STD_FOR_Z);
      nsScrFrequencyZ  = computeZScore(nsScrFrequencyPerMin, baselineNsScrFreqMean, baselineNsScrFreqStd, MIN_FREQ_STD_FOR_Z);

      // 스트레스 점수 변환
      sclStressScore            = zToStressScore(sclMeanZ);
      scrAmplitudeStressScore   = zToStressScore(scrAmplitudeMeanZ);
      nsScrFrequencyStressScore = zToStressScore(nsScrFrequencyZ);

      // 최종 GSR 점수: SCR이 검출된 경우에만 SCR 지표를 평균에 포함
      float gsrScoreSum  = 0.0;
      int   gsrScoreCount = 0;

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
      finalGsrStressScore = (gsrScoreCount > 0) ? gsrScoreSum / gsrScoreCount : NAN;

      // latest 변수 업데이트
      latestSclMean30s             = sclMean30s;
      latestScrAmplitudeMean30s    = scrAmplitudeMean30s;
      latestNsScrFrequencyPerMin   = nsScrFrequencyPerMin;
      latestSclMeanZ               = sclMeanZ;
      latestScrAmplitudeMeanZ      = scrAmplitudeMeanZ;
      latestNsScrFrequencyZ        = nsScrFrequencyZ;
      latestSclStressScore         = sclStressScore;
      latestScrAmplitudeStressScore= scrAmplitudeStressScore;
      latestNsScrFrequencyStressScore = nsScrFrequencyStressScore;
      latestFinalGsrStressScore    = finalGsrStressScore;
      latestFeatureReady           = 1;
      latestGsrScoreUpdateMs       = now;

      printCsvRow(now, sampleIndex, mode, rawGsr,
        medianGsr, sclBaseline, scrDetectBl, phasic, phasicPos,
        scrFlag, detectedScrTimeMs, detectedScrSampleIndex,
        sclMean30s, scrAmplitudeMean30s, nsScrFrequencyPerMin,
        sclMeanZ, scrAmplitudeMeanZ, nsScrFrequencyZ,
        sclStressScore, scrAmplitudeStressScore, nsScrFrequencyStressScore,
        finalGsrStressScore, featureReady, contactOk, validSampleFlag);

      // 윈도우 초기화 (다음 30초 준비)
      windowSclSum = 0.0; windowSampleCount = 0;
      windowScrAmpSum = 0.0; windowScrCount = 0;

      sampleIndex++;
      return;
    }
  }

  // PRINT_EVERY_SAMPLE 모드: 매 샘플 출력
  if (PRINT_EVERY_SAMPLE) {
    printCsvRow(now, sampleIndex, mode, rawGsr,
      medianGsr, sclBaseline, scrDetectBl, phasic, phasicPos,
      scrFlag, detectedScrTimeMs, detectedScrSampleIndex,
      sclMean30s, scrAmplitudeMean30s, nsScrFrequencyPerMin,
      sclMeanZ, scrAmplitudeMeanZ, nsScrFrequencyZ,
      sclStressScore, scrAmplitudeStressScore, nsScrFrequencyStressScore,
      finalGsrStressScore, featureReady, contactOk, validSampleFlag);
  }

  sampleIndex++;
}

} // namespace gsr


// ================================================================
// [PPG 모듈] MAX30102 기반 심박수·HRV 처리 및 스트레스 점수 계산
// ================================================================

// ----------------------------------------------------------------
// [PPG] 샘플링 설정
// ----------------------------------------------------------------
const float         FS                 = 100.0; // 샘플링 주파수 (Hz)
const unsigned long SAMPLE_INTERVAL_MS = 10;    // 샘플 주기 (ms)

// 측정 구간 길이 (ms)
const unsigned long STABILIZING_TIME_MS = 15000;  // 워밍업
const unsigned long CALIBRATION_TIME_MS = 60000;  // 베이스라인 수집
const unsigned long REST_TIME_MS        = 300000; // 안정 (label 0)
const unsigned long TRANSITION_TIME_MS  = 30000;  // 과도기 (라벨링 제외)
const unsigned long STRESS_TIME_MS      = 270000; // 스트레스 (label 1)
// 총 측정 시간: 15 + 60 + 300 + 30 + 270 = 675초 (11분 15초)

unsigned long startTime    = 0;
unsigned long lastSampleTime = 0;
unsigned long lastPrintTime  = 0;

// DONE 구간에서 완료 메시지를 한 번만 출력하기 위한 플래그
bool measurementDonePrinted = false;

const unsigned long PRINT_INTERVAL_MS = 1000; // CSV 출력 주기 (1초)

// ----------------------------------------------------------------
// [PPG] 손가락 미접촉 판단 설정
// ----------------------------------------------------------------
const long NO_FINGER_IR_THRESHOLD  = 10000; // IR 값이 이 미만이면 미접촉으로 의심
const int  NO_FINGER_CONFIRM_COUNT = 10;    // 연속 N회 이상 미접촉이면 확정
int consecutiveNoFinger = 0;
int noFingerCount       = 0;

// ----------------------------------------------------------------
// [PPG] Butterworth 밴드패스 필터 계수
// HPF 0.5 Hz + LPF 8 Hz, Fs = 100 Hz
// ----------------------------------------------------------------
// 고역통과필터 (HPF): 느린 기저선 드리프트 제거
float b_hp[3] = { 0.97803048, -1.95606096, 0.97803048 };
float a_hp[3] = { 1.0,        -1.95557824, 0.95654368 };

// 저역통과필터 (LPF): 고주파 잡음 제거
float b_lp[3] = { 0.0461318, 0.0922636, 0.0461318  };
float a_lp[3] = { 1.0,      -1.30728503, 0.49181224 };

// 필터 상태 변수 (IIR 필터는 이전 입력·출력값이 필요)
float hp_x1 = 0, hp_x2 = 0, hp_y1 = 0, hp_y2 = 0;
float lp_x1 = 0, lp_x2 = 0, lp_y1 = 0, lp_y2 = 0;

// ----------------------------------------------------------------
// [PPG] 피크 검출 및 IBI 설정
// ----------------------------------------------------------------
float ppg_prev2 = 0, ppg_prev1 = 0, ppg_curr = 0; // 3-샘플 슬라이딩 윈도우

float abs_ema = 0;                       // PPG 절댓값의 지수이동평균 (동적 임계값용)
const float ABS_EMA_ALPHA       = 0.01; // EMA 평활 계수
const float PEAK_THRESHOLD_RATIO= 0.35; // abs_ema의 35%를 피크 임계값으로 사용

unsigned long lastPeakTime = 0;
const unsigned long REFRACTORY_MS = 300; // 피크 검출 후 재검출 금지 시간 (ms)

float rawIbiMs   = NAN; // 피크 간격으로 계산된 원시 IBI
float validIbiMs = NAN; // 전처리 통과 후 유효 IBI

const float MIN_IBI_MS = 300.0;  // IBI 유효 범위 하한 (≈ 200 BPM)
const float MAX_IBI_MS = 1500.0; // IBI 유효 범위 상한 (≈ 40 BPM)

// 로컬 미디언 기반 이상치 제거
const int   MEDIAN_BUF_SIZE        = 7;
float medianBuf[MEDIAN_BUF_SIZE];
int   medianCount = 0, medianIndex = 0;
const float LOCAL_MEDIAN_TOLERANCE = 0.30; // 로컬 미디언 ±30% 범위 허용

// ----------------------------------------------------------------
// [PPG] HRV 계산용 IBI 버퍼
// ----------------------------------------------------------------
const int IBI_BUF_SIZE = 30;
float ibiBuf[IBI_BUF_SIZE];
int   ibiCount = 0, ibiIndex = 0;

float meanHR = NAN; // 평균 심박수 (BPM)
float sdnn   = NAN; // SDNN: IBI 표준편차 (ms)
float rmssd  = NAN; // RMSSD: 연속 IBI 차이의 RMS (ms)

// ----------------------------------------------------------------
// [PPG] 베이스라인 HRV 통계 (Welford 알고리즘으로 온라인 계산)
// ----------------------------------------------------------------
int baselineMetricCount = 0;

float baselineMeanHRMean = NAN, baselineMeanHRStd = NAN;
float baselineSDNNMean   = NAN, baselineSDNNStd   = NAN;
float baselineRMSSDMean  = NAN, baselineRMSSDStd  = NAN;

// Welford 누적 변수 (m: 평균, s: 분산 × n)
float meanHR_m = 0, meanHR_s = 0;
float sdnn_m   = 0, sdnn_s   = 0;
float rmssd_m  = 0, rmssd_s  = 0;

// ----------------------------------------------------------------
// [PPG] 베이스라인 IBI 변동 범위 (이상치 필터용)
// ----------------------------------------------------------------
int   baselineIbiCount = 0;
float baselineIbiMean  = NAN, baselineIbiStd = NAN;
float baselineIbi_m = 0, baselineIbi_s = 0;

// 마우스 기반 PPG 환경을 고려해 4 SD로 완화
const float BASELINE_IBI_STD_MULT = 4.0;

// ----------------------------------------------------------------
// [PPG] SQI (신호 품질 지수) 설정
// IBI 변동계수(CV)와 진폭 CV로 신호 품질을 평가한다.
// ----------------------------------------------------------------
const int   SQI_BUF_SIZE  = 10;
const int   SQI_MIN_COUNT = 5;    // 최소 5개 IBI가 있어야 SQI 계산

float sqiIbiBuf[SQI_BUF_SIZE], sqiAmpBuf[SQI_BUF_SIZE];
int   sqiCount = 0, sqiIndex = 0;

const float MAX_IBI_CV    = 0.45;  // IBI CV 허용 상한 (완화값)
const float MAX_AMP_CV    = 1.20;  // 진폭 CV 허용 상한 (완화값)
const float MIN_SQI_SCORE = 50.0;  // SQI가 이 이상이어야 유효 윈도우

float currentSQI    = NAN;
int   validWindowFlag = 0;

// ----------------------------------------------------------------
// [PPG] Z-score 기반 스트레스 점수 변수
// ----------------------------------------------------------------
float meanHrZ = NAN, sdnnZ = NAN, rmssdZ = NAN;
float meanHrStressScore = NAN, sdnnStressScore = NAN, rmssdStressScore = NAN;
float ppgAvgScore = NAN, hrvScore = NAN;
float finalPpgStressScore = NAN;
String finalPpgStressLevel = "";

const float MAX_Z_FOR_SCORE     = 3.0;
const float NORMAL_THRESHOLD    = 33.3; // 이 미만 → Normal
const float STRESS_THRESHOLD    = 66.7; // 이 이상 → Stress

const int   MIN_BASELINE_METRIC_COUNT_FOR_Z = 20; // Z-score 계산에 필요한 최소 베이스라인 샘플 수

// 표준편차 최솟값 (Z-score 폭주 방지)
const float MIN_MEAN_HR_STD_FOR_Z = 3.0;
const float MIN_SDNN_STD_FOR_Z    = 5.0;
const float MIN_RMSSD_STD_FOR_Z   = 5.0;


// ================================================================
// [PPG] 유틸리티 함수
// ================================================================

// NAN이면 빈 문자열, 아니면 소수점 digits자리로 출력
void printFloatOrEmpty(float x, int digits) {
  if (isnan(x)) Serial.print("");
  else           Serial.print(x, digits);
}

// 값을 [low, high]로 클램프
float clampFloat(float x, float low, float high) {
  if (x < low)  return low;
  if (x > high) return high;
  return x;
}


// ================================================================
// [PPG] 밴드패스 필터 함수 (HPF → LPF 직렬 적용)
// ================================================================
float applyHPF(float x) {
  float y = b_hp[0]*x + b_hp[1]*hp_x1 + b_hp[2]*hp_x2
          - a_hp[1]*hp_y1 - a_hp[2]*hp_y2;
  hp_x2 = hp_x1; hp_x1 = x;
  hp_y2 = hp_y1; hp_y1 = y;
  return y;
}

float applyLPF(float x) {
  float y = b_lp[0]*x + b_lp[1]*lp_x1 + b_lp[2]*lp_x2
          - a_lp[1]*lp_y1 - a_lp[2]*lp_y2;
  lp_x2 = lp_x1; lp_x1 = x;
  lp_y2 = lp_y1; lp_y1 = y;
  return y;
}

float applyBandpass(float x) {
  return applyLPF(applyHPF(x)); // HPF로 저주파 제거 후 LPF로 고주파 제거
}


// ================================================================
// [PPG] 통계 함수
// ================================================================
float calcMean(float arr[], int n) {
  if (n <= 0) return NAN;
  float sum = 0;
  for (int i = 0; i < n; i++) sum += arr[i];
  return sum / n;
}

float calcStd(float arr[], int n) {
  if (n < 2) return NAN;
  float m = calcMean(arr, n), sumSq = 0;
  for (int i = 0; i < n; i++) {
    float d = arr[i] - m;
    sumSq += d * d;
  }
  return sqrt(sumSq / (n - 1));
}

float calcMedian(float arr[], int n) {
  if (n <= 0) return NAN;
  float temp[MEDIAN_BUF_SIZE];
  for (int i = 0; i < n; i++) temp[i] = arr[i];

  // 버블 정렬로 오름차순 정렬
  for (int i = 0; i < n - 1; i++)
    for (int j = i + 1; j < n; j++)
      if (temp[j] < temp[i]) { float t = temp[i]; temp[i] = temp[j]; temp[j] = t; }

  return (n % 2 == 1) ? temp[n / 2] : (temp[n/2 - 1] + temp[n/2]) / 2.0;
}

// 변동계수 = 표준편차 / 평균 (신호 안정성 지표)
float calcCV(float arr[], int n) {
  if (n < 2) return NAN;
  float m = calcMean(arr, n), s = calcStd(arr, n);
  if (isnan(m) || m == 0 || isnan(s)) return NAN;
  return s / m;
}


// ================================================================
// [PPG] Welford 온라인 베이스라인 누적 함수
// 배열 없이 평균·분산을 실시간으로 계산하는 수치적으로 안정적인 방법
// ================================================================
void updateWelford(float x, int n, float &m, float &s) {
  float delta  = x - m;
  m += delta / n;
  float delta2 = x - m;
  s += delta * delta2;
}

// HRV 지표(meanHR, SDNN, RMSSD)를 베이스라인 누적값에 추가
void updateBaselineMetrics() {
  if (isnan(meanHR) || isnan(sdnn) || isnan(rmssd)) return;

  baselineMetricCount++;
  updateWelford(meanHR, baselineMetricCount, meanHR_m, meanHR_s);
  updateWelford(sdnn,   baselineMetricCount, sdnn_m,   sdnn_s);
  updateWelford(rmssd,  baselineMetricCount, rmssd_m,  rmssd_s);

  baselineMeanHRMean = meanHR_m;
  baselineSDNNMean   = sdnn_m;
  baselineRMSSDMean  = rmssd_m;

  if (baselineMetricCount >= 2) {
    baselineMeanHRStd = sqrt(meanHR_s / (baselineMetricCount - 1));
    baselineSDNNStd   = sqrt(sdnn_s   / (baselineMetricCount - 1));
    baselineRMSSDStd  = sqrt(rmssd_s  / (baselineMetricCount - 1));
  }
}

// CALIBRATION 구간 IBI를 베이스라인 범위 계산에 누적
void updateBaselineIbi(float ibi) {
  if (isnan(ibi)) return;
  baselineIbiCount++;
  float delta  = ibi - baselineIbi_m;
  baselineIbi_m += delta / baselineIbiCount;
  float delta2  = ibi - baselineIbi_m;
  baselineIbi_s += delta * delta2;
  baselineIbiMean = baselineIbi_m;
  if (baselineIbiCount >= 2)
    baselineIbiStd = sqrt(baselineIbi_s / (baselineIbiCount - 1));
}


// ================================================================
// [PPG] 원형 버퍼 관리 함수
// ================================================================
void addMedianBuffer(float ibi) {
  medianBuf[medianIndex] = ibi;
  medianIndex = (medianIndex + 1) % MEDIAN_BUF_SIZE;
  if (medianCount < MEDIAN_BUF_SIZE) medianCount++;
}

void addIbiBuffer(float ibi) {
  ibiBuf[ibiIndex] = ibi;
  ibiIndex = (ibiIndex + 1) % IBI_BUF_SIZE;
  if (ibiCount < IBI_BUF_SIZE) ibiCount++;
}

void addSqiBuffer(float ibi, float amp) {
  sqiIbiBuf[sqiIndex] = ibi;
  sqiAmpBuf[sqiIndex] = abs(amp);
  sqiIndex = (sqiIndex + 1) % SQI_BUF_SIZE;
  if (sqiCount < SQI_BUF_SIZE) sqiCount++;
}


// ================================================================
// [PPG] IBI 유효성 검사 함수
// ================================================================

// 1차 검사: 절대 범위 + 로컬 미디언 ±30% 범위
bool isBasicValidIbi(float ibi) {
  if (isnan(ibi)) return false;
  if (ibi < MIN_IBI_MS || ibi > MAX_IBI_MS) return false;

  if (medianCount >= 3) {
    float med   = calcMedian(medianBuf, medianCount);
    float lower = med * (1.0 - LOCAL_MEDIAN_TOLERANCE);
    float upper = med * (1.0 + LOCAL_MEDIAN_TOLERANCE);
    if (ibi < lower || ibi > upper) return false;
  }
  return true;
}

// 2차 검사: CALIBRATION에서 학습한 베이스라인 IBI 범위 (±4 SD)
bool isBaselineRangeValidIbi(float ibi) {
  if (baselineIbiCount >= 10 &&
      !isnan(baselineIbiMean) && !isnan(baselineIbiStd) && baselineIbiStd > 0) {
    float lower = max(baselineIbiMean - BASELINE_IBI_STD_MULT * baselineIbiStd, MIN_IBI_MS);
    float upper = min(baselineIbiMean + BASELINE_IBI_STD_MULT * baselineIbiStd, MAX_IBI_MS);
    if (ibi < lower || ibi > upper) return false;
  }
  return true;
}


// ================================================================
// [PPG] SQI (신호 품질 지수) 계산
// IBI 변동계수와 진폭 변동계수를 가중평균해 0~100 점수로 환산
// ================================================================
float calculateSQI() {
  if (sqiCount < SQI_MIN_COUNT) { validWindowFlag = 0; return NAN; }

  float tempIbi[SQI_BUF_SIZE], tempAmp[SQI_BUF_SIZE];
  for (int i = 0; i < sqiCount; i++) {
    int idx    = (sqiIndex - sqiCount + i + SQI_BUF_SIZE) % SQI_BUF_SIZE;
    tempIbi[i] = sqiIbiBuf[idx];
    tempAmp[i] = sqiAmpBuf[idx];
  }

  float ibiCV = calcCV(tempIbi, sqiCount);
  float ampCV = calcCV(tempAmp, sqiCount);
  if (isnan(ibiCV) || isnan(ampCV)) { validWindowFlag = 0; return NAN; }

  // CV가 작을수록 높은 점수 (신호가 안정적)
  float ibiScore = constrain(100.0 * (1.0 - ibiCV / MAX_IBI_CV), 0.0, 100.0);
  float ampScore = constrain(100.0 * (1.0 - ampCV / MAX_AMP_CV), 0.0, 100.0);
  float sqiScore = 0.6 * ibiScore + 0.4 * ampScore; // IBI에 더 높은 가중치

  validWindowFlag = (sqiScore >= MIN_SQI_SCORE) ? 1 : 0;
  return sqiScore;
}


// ================================================================
// [PPG] HRV 지표 계산
// 최근 N개의 IBI 버퍼로 meanHR / SDNN / RMSSD를 계산한다.
// ================================================================
void updateHrvMetrics() {
  if (ibiCount < 5) { meanHR = sdnn = rmssd = NAN; return; }

  float temp[IBI_BUF_SIZE];
  for (int i = 0; i < ibiCount; i++) {
    int idx  = (ibiIndex - ibiCount + i + IBI_BUF_SIZE) % IBI_BUF_SIZE;
    temp[i]  = ibiBuf[idx];
  }

  float meanIbi = calcMean(temp, ibiCount);
  meanHR = 60000.0 / meanIbi; // ms → BPM
  sdnn   = calcStd(temp, ibiCount);

  // RMSSD: 연속 IBI 차이의 제곱평균제곱근
  float sumDiffSq = 0;
  int   diffCount = 0;
  for (int i = 1; i < ibiCount; i++) {
    float diff = temp[i] - temp[i - 1];
    sumDiffSq += diff * diff;
    diffCount++;
  }
  rmssd = (diffCount > 0) ? sqrt(sumDiffSq / diffCount) : NAN;
}


// ================================================================
// [PPG] 스트레스 레벨 분류 및 점수 계산
// ================================================================

// 0~100 점수를 Normal / Mild / Stress로 분류
String classifyStressLevel(float score) {
  if (isnan(score)) return "";
  if (score >= STRESS_THRESHOLD) return "Stress";
  if (score >= NORMAL_THRESHOLD) return "Mild";
  return "Normal";
}

void resetPpgStressScore() {
  meanHrZ = sdnnZ = rmssdZ = NAN;
  meanHrStressScore = sdnnStressScore = rmssdStressScore = NAN;
  ppgAvgScore = hrvScore = finalPpgStressScore = NAN;
  finalPpgStressLevel = "";
}

// REST/STRESS 구간에서 Z-score를 계산하고 최종 PPG 스트레스 점수를 결정한다.
void updatePpgStressScore(String mode) {
  if (mode != "REST" && mode != "STRESS") { resetPpgStressScore(); return; }

  // 베이스라인이 충분하지 않으면 보류
  if (baselineMetricCount < MIN_BASELINE_METRIC_COUNT_FOR_Z ||
      isnan(meanHR) || isnan(sdnn) || isnan(rmssd) ||
      isnan(baselineMeanHRMean) || isnan(baselineMeanHRStd) ||
      isnan(baselineSDNNMean)   || isnan(baselineSDNNStd) ||
      isnan(baselineRMSSDMean)  || isnan(baselineRMSSDStd)) {
    resetPpgStressScore();
    return;
  }

  // 최소 표준편차 적용 후 Z-score 계산
  float safeMeanHrStd = max(baselineMeanHRStd, MIN_MEAN_HR_STD_FOR_Z);
  float safeSdnnStd   = max(baselineSDNNStd,   MIN_SDNN_STD_FOR_Z);
  float safeRmssdStd  = max(baselineRMSSDStd,  MIN_RMSSD_STD_FOR_Z);

  meanHrZ = (meanHR - baselineMeanHRMean) / safeMeanHrStd; // 증가 → 스트레스
  sdnnZ   = (sdnn   - baselineSDNNMean)   / safeSdnnStd;   // 감소 → 스트레스
  rmssdZ  = (rmssd  - baselineRMSSDMean)  / safeRmssdStd;  // 감소 → 스트레스

  // 0~100 점수 변환 (SDNN·RMSSD는 부호 반전: 낮을수록 스트레스)
  meanHrStressScore = clampFloat( (meanHrZ / MAX_Z_FOR_SCORE) * 100.0, 0.0, 100.0);
  sdnnStressScore   = clampFloat((-sdnnZ   / MAX_Z_FOR_SCORE) * 100.0, 0.0, 100.0);
  rmssdStressScore  = clampFloat((-rmssdZ  / MAX_Z_FOR_SCORE) * 100.0, 0.0, 100.0);

  ppgAvgScore = (meanHrStressScore + sdnnStressScore + rmssdStressScore) / 3.0;
  hrvScore    = (sdnnStressScore + rmssdStressScore) / 2.0;

  // HRV 두 지표가 모두 Stress/Mild 임계값을 넘으면 상향 조정
  bool rmssdStress = rmssdStressScore >= STRESS_THRESHOLD;
  bool sdnnStress  = sdnnStressScore  >= STRESS_THRESHOLD;
  bool rmssdMild   = rmssdStressScore >= NORMAL_THRESHOLD;
  bool sdnnMild    = sdnnStressScore  >= NORMAL_THRESHOLD;

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


// ================================================================
// [PPG] 모드 및 라벨 함수
// ================================================================

// 경과 시간과 손가락 접촉 여부로 현재 측정 모드를 반환
String getMode(unsigned long elapsedMs, bool noFinger) {
  if (noFinger) return "NO_FINGER";

  unsigned long t1 = STABILIZING_TIME_MS;
  unsigned long t2 = t1 + CALIBRATION_TIME_MS;
  unsigned long t3 = t2 + REST_TIME_MS;
  unsigned long t4 = t3 + TRANSITION_TIME_MS;
  unsigned long t5 = t4 + STRESS_TIME_MS;

  if (elapsedMs < t1) return "STABILIZING";
  if (elapsedMs < t2) return "CALIBRATION";
  if (elapsedMs < t3) return "REST";
  if (elapsedMs < t4) return "TRANSITION";
  if (elapsedMs < t5) return "STRESS";
  return "DONE";
}

// 성능 평가용 클래스 라벨 (-1: 제외, 0: 안정, 1: 스트레스)
int getClassLabel(String mode) {
  if (mode == "REST")   return 0;
  if (mode == "STRESS") return 1;
  return -1;
}

// 라벨이 유효한지 여부 (0 또는 1이면 유효)
int getLabelValid(String mode) {
  int label = getClassLabel(mode);
  return (label == 0 || label == 1) ? 1 : 0;
}


// ================================================================
// [통합] CSV 출력 함수
// PPG + GSR + 통합 스트레스 점수를 한 줄로 출력한다.
// ================================================================
void printCombinedCsvRow(unsigned long elapsedMs, String mode, long irRaw, float filteredPpg) {

  // ── 통합 스트레스 점수 계산 ──────────────────────────────────
  // PPG와 GSR 각각의 유효성을 확인하고, 둘 다 유효하면 평균, 하나만 유효하면 그 값을 사용
  bool ppgReady = (mode == "REST" || mode == "STRESS") &&
                  (validWindowFlag == 1) &&
                  !isnan(finalPpgStressScore);

  bool gsrReady = (gsr::latestMode == "REST" || gsr::latestMode == "STRESS") &&
                  (gsr::latestContactOk == 1) &&
                  (gsr::latestValidSampleFlag == 1) &&
                  gsr::isLatestScoreFresh(millis()) &&
                  !isnan(gsr::latestFinalGsrStressScore);

  float integratedStressScore = NAN;
  String integratedStressLevel = "";

  float scoreSum  = 0.0;
  int   scoreCount = 0;
  if (ppgReady) { scoreSum += finalPpgStressScore;            scoreCount++; }
  if (gsrReady) { scoreSum += gsr::latestFinalGsrStressScore; scoreCount++; }

  if (scoreCount > 0) {
    integratedStressScore = scoreSum / scoreCount;
    integratedStressLevel = classifyStressLevel(integratedStressScore);
  }

  // ── CSV 출력 ─────────────────────────────────────────────────
  // PPG 관련 컬럼
  Serial.print(elapsedMs);                    Serial.print(",");
  Serial.print(mode);                         Serial.print(",");
  Serial.print(irRaw);                        Serial.print(",");
  Serial.print(getClassLabel(mode));          Serial.print(",");
  Serial.print(getLabelValid(mode));          Serial.print(",");
  printFloatOrEmpty(filteredPpg, 4);          Serial.print(",");
  printFloatOrEmpty(rawIbiMs, 2);             Serial.print(",");
  printFloatOrEmpty(validIbiMs, 2);           Serial.print(",");
  printFloatOrEmpty(meanHR, 2);               Serial.print(",");
  printFloatOrEmpty(sdnn, 2);                 Serial.print(",");
  printFloatOrEmpty(rmssd, 2);               Serial.print(",");
  printFloatOrEmpty(baselineMeanHRMean, 2);   Serial.print(",");
  printFloatOrEmpty(baselineMeanHRStd, 2);    Serial.print(",");
  printFloatOrEmpty(baselineSDNNMean, 2);     Serial.print(",");
  printFloatOrEmpty(baselineSDNNStd, 2);      Serial.print(",");
  printFloatOrEmpty(baselineRMSSDMean, 2);    Serial.print(",");
  printFloatOrEmpty(baselineRMSSDStd, 2);     Serial.print(",");
  printFloatOrEmpty(baselineIbiMean, 2);      Serial.print(",");
  printFloatOrEmpty(baselineIbiStd, 2);       Serial.print(",");
  printFloatOrEmpty(currentSQI, 2);           Serial.print(",");
  Serial.print(validWindowFlag);              Serial.print(",");
  printFloatOrEmpty(meanHrZ, 4);              Serial.print(",");
  printFloatOrEmpty(sdnnZ, 4);                Serial.print(",");
  printFloatOrEmpty(rmssdZ, 4);              Serial.print(",");
  printFloatOrEmpty(meanHrStressScore, 2);    Serial.print(",");
  printFloatOrEmpty(sdnnStressScore, 2);      Serial.print(",");
  printFloatOrEmpty(rmssdStressScore, 2);     Serial.print(",");
  printFloatOrEmpty(ppgAvgScore, 2);          Serial.print(",");
  printFloatOrEmpty(hrvScore, 2);             Serial.print(",");
  printFloatOrEmpty(finalPpgStressScore, 2);  Serial.print(",");
  Serial.print(finalPpgStressLevel);          Serial.print(",");
  Serial.print(ibiCount);                     Serial.print(",");
  Serial.print(baselineMetricCount);          Serial.print(",");
  Serial.print(noFingerCount);                Serial.print(",");

  // GSR 관련 컬럼
  Serial.print(gsr::latestMode);                        Serial.print(",");
  Serial.print(gsr::latestSampleIndex);                 Serial.print(",");
  Serial.print(gsr::latestRawGsr);                      Serial.print(",");
  printFloatOrEmpty(gsr::latestMedianGsr, 2);            Serial.print(",");
  printFloatOrEmpty(gsr::latestSclBaseline, 2);          Serial.print(",");
  printFloatOrEmpty(gsr::latestScrDetectBaseline, 2);    Serial.print(",");
  printFloatOrEmpty(gsr::latestPhasic, 2);               Serial.print(",");
  printFloatOrEmpty(gsr::latestPhasicPos, 2);            Serial.print(",");
  printFloatOrEmpty(gsr::scrThreshold, 2);               Serial.print(",");
  printFloatOrEmpty(gsr::MIN_SCR_THRESHOLD_ADC, 2);      Serial.print(",");
  printFloatOrEmpty(gsr::MIN_SCR_RISE_ADC, 2);           Serial.print(",");
  printFloatOrEmpty(gsr::latestSclMean30s, 2);           Serial.print(",");
  printFloatOrEmpty(gsr::latestScrAmplitudeMean30s, 2);  Serial.print(",");
  printFloatOrEmpty(gsr::latestNsScrFrequencyPerMin, 2); Serial.print(",");
  printFloatOrEmpty(gsr::latestSclMeanZ, 4);             Serial.print(",");
  printFloatOrEmpty(gsr::latestScrAmplitudeMeanZ, 4);    Serial.print(",");
  printFloatOrEmpty(gsr::latestNsScrFrequencyZ, 4);      Serial.print(",");
  printFloatOrEmpty(gsr::latestSclStressScore, 2);       Serial.print(",");
  printFloatOrEmpty(gsr::latestScrAmplitudeStressScore, 2); Serial.print(",");
  printFloatOrEmpty(gsr::latestNsScrFrequencyStressScore, 2); Serial.print(",");
  printFloatOrEmpty(gsr::latestFinalGsrStressScore, 2);  Serial.print(",");
  Serial.print(gsr::latestFeatureReady);                 Serial.print(",");
  Serial.print(gsr::latestContactOk);                    Serial.print(",");
  Serial.print(gsr::latestValidSampleFlag);              Serial.print(",");

  // 통합 스트레스 점수
  printFloatOrEmpty(integratedStressScore, 2);
  Serial.print(",");
  Serial.println(integratedStressLevel);
}


// ================================================================
// [시스템] 초기화 함수
// ================================================================

// PPG 관련 모든 변수를 초기값으로 리셋
void resetPpgVariables() {
  hp_x1 = hp_x2 = hp_y1 = hp_y2 = 0;
  lp_x1 = lp_x2 = lp_y1 = lp_y2 = 0;

  ppg_prev2 = ppg_prev1 = ppg_curr = 0;
  abs_ema = 0; lastPeakTime = 0;
  rawIbiMs = validIbiMs = NAN;

  for (int i = 0; i < MEDIAN_BUF_SIZE; i++) medianBuf[i] = 0;
  medianCount = medianIndex = 0;

  for (int i = 0; i < IBI_BUF_SIZE; i++) ibiBuf[i] = 0;
  ibiCount = ibiIndex = 0;
  meanHR = sdnn = rmssd = NAN;

  baselineMetricCount = 0;
  baselineMeanHRMean = baselineMeanHRStd = NAN;
  baselineSDNNMean   = baselineSDNNStd   = NAN;
  baselineRMSSDMean  = baselineRMSSDStd  = NAN;
  meanHR_m = meanHR_s = sdnn_m = sdnn_s = rmssd_m = rmssd_s = 0;

  baselineIbiCount = 0;
  baselineIbiMean = baselineIbiStd = NAN;
  baselineIbi_m = baselineIbi_s = 0;

  for (int i = 0; i < SQI_BUF_SIZE; i++) { sqiIbiBuf[i] = sqiAmpBuf[i] = 0; }
  sqiCount = sqiIndex = 0;
  currentSQI = NAN; validWindowFlag = 0;

  meanHrZ = sdnnZ = rmssdZ = NAN;
  meanHrStressScore = sdnnStressScore = rmssdStressScore = NAN;
  ppgAvgScore = hrvScore = finalPpgStressScore = NAN;
  finalPpgStressLevel = "";

  consecutiveNoFinger = noFingerCount = 0;
}

// 전체 시스템 초기화 후 측정 시작 (재측정 포함)
void startMeasurement() {
  resetPpgVariables();
  gsr::resetAll();

  unsigned long t0 = millis();
  startTime           = t0;
  lastSampleTime      = t0;
  lastPrintTime       = t0;
  gsr::lastSampleTime = t0;
  measurementDonePrinted = false;

  // CSV 헤더 출력
  Serial.println(
    "time_ms,ppg_mode,ir_raw,class_label,label_valid,filtered_ppg,"
    "ibi_ms,valid_ibi_ms,mean_hr,sdnn,rmssd,"
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
    "phasic,phasic_pos,gsr_scr_threshold,gsr_min_scr_threshold_adc,gsr_min_scr_rise_adc,"
    "scl_mean_30s,scr_amplitude_mean_30s,ns_scr_frequency_per_min,"
    "scl_mean_z,scr_amplitude_mean_z,ns_scr_frequency_z,"
    "scl_stress_score,scr_amplitude_stress_score,ns_scr_frequency_stress_score,"
    "final_gsr_stress_score,gsr_feature_ready,gsr_contact_ok,gsr_valid_sample_flag,"
    "integrated_stress_score,integrated_stress_level"
  );
}


// ================================================================
// [시스템] setup / loop
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin();

  // MAX30102 센서 초기화
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR,MAX30102_NOT_FOUND");
    while (1);
  }

  // 센서 파라미터 설정
  particleSensor.setup(
    0x5F,  // LED 밝기
    4,     // 샘플 평균 횟수
    2,     // LED 모드 (Red + IR)
    100,   // 샘플링 레이트 (Hz)
    411,   // 펄스 폭 (μs)
    16384  // ADC 범위
  );
  particleSensor.setPulseAmplitudeRed(0x5F);
  particleSensor.setPulseAmplitudeIR(0x5F);

  gsr::setup();

  // Python에서 'S' 신호를 받을 때까지 대기
  Serial.println("READY");
  while (true) {
    if (Serial.available() && Serial.read() == 'S') break;
  }

  startMeasurement(); // 'S' 수신 시점부터 타이머 시작
}

void loop() {
  // 'R' 수신 시 전체 리셋 후 재측정
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'R' || cmd == 'r') {
      Serial.println("RESET");
      delay(100);
      startMeasurement();
      return;
    }
  }

  // GSR 모듈 루프 (10Hz 자체 타이밍 내장)
  gsr::loop();

  unsigned long now = millis();

  // PPG는 10ms(100Hz) 간격으로 샘플링
  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) return;
  lastSampleTime = now;

  unsigned long elapsedMs = now - startTime;
  long irRaw = particleSensor.getIR();

  // ── 손가락 접촉 판정 ─────────────────────────────────────────
  if (irRaw <= 0 || irRaw < NO_FINGER_IR_THRESHOLD) consecutiveNoFinger++;
  else                                               consecutiveNoFinger = 0;

  bool noFinger = (consecutiveNoFinger >= NO_FINGER_CONFIRM_COUNT);
  if (noFinger) noFingerCount++;

  String mode = getMode(elapsedMs, noFinger);

  // ── 측정 완료 처리 ───────────────────────────────────────────
  // DONE 이후에도 loop를 유지해 'R' 명령으로 재측정 가능
  if (mode == "DONE") {
    if (!measurementDonePrinted) {
      Serial.println("MEASUREMENT_DONE");
      measurementDonePrinted = true;
    }
    return;
  }

  // ── PPG 신호 처리 ────────────────────────────────────────────
  float filteredPpg = NAN;

  if (!noFinger && irRaw > 0) {
    filteredPpg = applyBandpass((float)irRaw);

    if (mode != "STABILIZING") {
      // 슬라이딩 윈도우 밀기
      ppg_prev2 = ppg_prev1;
      ppg_prev1 = ppg_curr;
      ppg_curr  = filteredPpg;

      // 동적 임계값: abs_ema × 0.35
      abs_ema = (1.0 - ABS_EMA_ALPHA) * abs_ema + ABS_EMA_ALPHA * abs(filteredPpg);
      float peakThreshold = abs_ema * PEAK_THRESHOLD_RATIO;

      // 피크 타임: 실제 봉우리는 한 샘플 전 (prev1)
      unsigned long peakTimeMs = (now >= SAMPLE_INTERVAL_MS) ? (now - SAMPLE_INTERVAL_MS) : now;

      bool isPeak = (ppg_prev1 > ppg_prev2) &&
                    (ppg_prev1 > ppg_curr)  &&
                    (ppg_prev1 > peakThreshold) &&
                    (peakTimeMs - lastPeakTime > REFRACTORY_MS);

      if (isPeak) {
        if (lastPeakTime > 0) {
          rawIbiMs = (float)(peakTimeMs - lastPeakTime);
          float peakAmp = ppg_prev1;

          // IBI 검증 파이프라인: basic → 로컬 미디언 → 베이스라인 범위 → SQI
          if (isBasicValidIbi(rawIbiMs)) {
            addMedianBuffer(rawIbiMs);

            if (mode == "CALIBRATION") updateBaselineIbi(rawIbiMs);

            if (isBaselineRangeValidIbi(rawIbiMs)) {
              addSqiBuffer(rawIbiMs, peakAmp);
              currentSQI = calculateSQI();

              if (validWindowFlag == 1) {
                validIbiMs = rawIbiMs;
                addIbiBuffer(validIbiMs);
                updateHrvMetrics();

                if (mode == "CALIBRATION") updateBaselineMetrics();
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
    filteredPpg = validIbiMs = NAN;
  }

  // ── 1초마다 CSV 출력 ─────────────────────────────────────────
  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;

    updatePpgStressScore(mode);              // PPG 스트레스 점수 갱신
    printCombinedCsvRow(elapsedMs, mode, irRaw, filteredPpg); // 통합 CSV 출력

    // 다음 1초 구간을 위해 IBI 리셋
    rawIbiMs = validIbiMs = NAN;
  }
}
