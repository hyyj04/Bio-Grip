import csv
import time
from datetime import datetime
from pathlib import Path

import serial
import pandas as pd
import matplotlib.pyplot as plt


# ==============================
# 1. 사용자 설정
# ==============================
SERIAL_PORT = "COM9"
BAUD_RATE = 115200

# None이면 사용자가 Ctrl+C로 직접 종료
RECORD_SECONDS = None

OUTPUT_DIR = Path("ppg_gsr_integrated_results")
OUTPUT_DIR.mkdir(exist_ok=True)

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
OUTPUT_CSV = OUTPUT_DIR / f"ppg_gsr_integrated_{timestamp}.csv"

SAVE_GRAPHS = True

# 아두이노에서 유효하지 않은 IBI를 나타내는 더미값(0 이하는 무효 처리)
INVALID_IBI_THRESHOLD = 0

# Arduino 헤더를 놓쳤을 때도 저장을 시작할지 여부
# 이전 로그처럼 헤더가 이미 지나간 뒤 데이터만 들어오는 경우를 대비
START_WITH_BUILTIN_HEADER_IF_DATA_COMES = True


# ==============================
# 2. 통합 Arduino 출력 컬럼
# ==============================
HEADER = [
    "time_ms",
    "ppg_mode",
    "ir_raw",
    "filtered_ppg",
    "ibi_ms",
    "valid_ibi_ms",
    "mean_hr",
    "sdnn",
    "rmssd",
    "baseline_mean_hr_mean",
    "baseline_mean_hr_std",
    "baseline_sdnn_mean",
    "baseline_sdnn_std",
    "baseline_rmssd_mean",
    "baseline_rmssd_std",
    "baseline_ibi_mean",
    "baseline_ibi_std",
    "sqi_score",
    "valid_window_flag",
    "mean_hr_z",
    "sdnn_z",
    "rmssd_z",
    "mean_hr_stress_score",
    "sdnn_stress_score",
    "rmssd_stress_score",
    "ppg_avg_score",
    "hrv_score",
    "final_ppg_stress_score",
    "final_ppg_stress_level",
    "ibi_count",
    "baseline_count",
    "no_finger_count",
    "gsr_mode",
    "gsr_sample_index",
    "raw_gsr",
    "median_gsr",
    "scl_baseline",
    "scr_detect_baseline",
    "phasic",
    "phasic_pos",
    "scl_mean_30s",
    "scr_amplitude_mean_30s",
    "ns_scr_frequency_per_min",
    "scl_mean_z",
    "scr_amplitude_mean_z",
    "ns_scr_frequency_z",
    "scl_stress_score",
    "scr_amplitude_stress_score",
    "ns_scr_frequency_stress_score",
    "final_gsr_stress_score",
    "gsr_feature_ready",
    "gsr_contact_ok",
    "gsr_valid_sample_flag",
    "integrated_stress_score",
    "integrated_stress_level",
]

EXTRA_COLS = [
    "pc_elapsed_sec",
    "analysis_ppg_mode",
    "analysis_gsr_mode",
]

FULL_HEADER = HEADER + EXTRA_COLS


# ==============================
# 3. mode 정리 함수
# ==============================
def normalize_ppg_mode(arduino_mode: str) -> str:
    """
    보고서/분석용 PPG mode를 STABILIZING / BASELINE / MONITORING으로 단순화한다.
    Arduino에서 CALIBRATION으로 넘어오면 BASELINE으로 저장한다.
    """
    mode_map = {
        "STABILIZING": "STABILIZING",
        "CALIBRATION": "BASELINE",
        "BASELINE": "BASELINE",
        "MONITORING": "MONITORING",
        "NO_FINGER": "NO_FINGER",
    }
    return mode_map.get(arduino_mode, arduino_mode)


def normalize_gsr_mode(arduino_mode: str) -> str:
    """
    GSR mode도 분석용으로 정리한다.
    통합 코드에서는 CALIBRATION / MEASUREMENT / NO_CONTACT 등이 들어올 수 있다.
    """
    mode_map = {
        "WARMUP": "WARMUP",
        "CALIBRATION": "BASELINE",
        "BASELINE": "BASELINE",
        "MEASUREMENT": "MEASUREMENT",
        "MONITORING": "MEASUREMENT",
        "NO_CONTACT": "NO_CONTACT",
    }
    return mode_map.get(arduino_mode, arduino_mode)


def clean_serial_line(line: str) -> str:
    """
    혹시 이전 디버그 출력처럼 '헤더 대기 중:'이 붙은 줄을 복사/재사용해도 처리되도록 방어.
    실제 Serial에서는 보통 필요 없다.
    """
    line = line.strip().replace("\ufeff", "")
    for prefix in ["헤더 대기 중:", "Arduino header 수신:"]:
        if line.startswith(prefix):
            line = line[len(prefix):].strip()
    return line


def looks_like_data_row(values: list[str]) -> bool:
    if len(values) != len(HEADER):
        return False
    try:
        int(float(values[0]))
    except ValueError:
        return False
    return True


# ==============================
# 4. 실행 정보 출력
# ==============================
print("====================================")
print("PPG + GSR 통합 Serial CSV 저장 시작")
print(f"PORT      : {SERIAL_PORT}")
print(f"BAUD RATE : {BAUD_RATE}")
print(f"OUTPUT    : {OUTPUT_CSV}")
print("종료 방법 : Ctrl + C")
print("주의      : Arduino IDE Serial Monitor / Serial Plotter는 닫아 주세요.")
print("====================================")


# ==============================
# 5. CSV 파일 먼저 생성
# ==============================
with open(OUTPUT_CSV, "w", newline="", encoding="utf-8-sig") as f:
    writer = csv.writer(f)
    writer.writerow(FULL_HEADER)

print("CSV 파일 생성 완료")
print(f"저장 경로: {OUTPUT_CSV}")


# ==============================
# 6. Serial 연결
# ==============================
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except serial.SerialException as e:
    print("\nSerial 포트를 열 수 없습니다.")
    print("확인:")
    print("1. Arduino IDE 시리얼 모니터가 닫혀 있는지")
    print("2. COM 포트 번호가 맞는지")
    print("3. USB 케이블이 연결되어 있는지")
    raise e

# Arduino reset 이후 첫 출력 대기.
# reset_input_buffer()는 일부 환경에서 Arduino 헤더를 지워버릴 수 있어 사용하지 않는다.
time.sleep(2)

print("\nSerial 연결 완료")
print("실시간 수신값을 표시하면서 CSV 저장을 시작합니다.")
print("측정을 중단하려면 Ctrl + C를 누르세요.\n")

rows = []
empty_count = 0
bad_count = 0
header_seen = False
start_time = time.time()


# ==============================
# 7. Serial 데이터 수신 + 터미널 출력 + CSV 저장
# ==============================
try:
    with open(OUTPUT_CSV, "a", newline="", encoding="utf-8-sig") as csv_file:
        writer = csv.writer(csv_file)

        while True:
            raw = ser.readline()
            pc_elapsed_sec = time.time() - start_time

            if RECORD_SECONDS is not None and pc_elapsed_sec >= RECORD_SECONDS:
                print("\n측정 시간 종료")
                break

            if not raw:
                empty_count += 1
                if empty_count % 5 == 0:
                    print(f"{int(pc_elapsed_sec)} sec / 시리얼 데이터 대기 중... empty_count={empty_count}")
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            line = clean_serial_line(line)

            if not line:
                empty_count += 1
                if empty_count % 5 == 0:
                    print(f"{int(pc_elapsed_sec)} sec / 빈 줄 수신 중... empty_count={empty_count}")
                continue

            if line.startswith("ERROR"):
                print("Arduino ERROR:", line)
                continue

            # Arduino가 출력한 CSV 헤더 감지
            if line.startswith("time_ms"):
                serial_header = [c.strip() for c in line.split(",")]
                if serial_header != HEADER:
                    print("\n[경고] Arduino 헤더가 Python HEADER와 다릅니다.")
                    print(f"Arduino 컬럼 수: {len(serial_header)}")
                    print(f"Python  컬럼 수: {len(HEADER)}")
                    print("저장은 Python 내장 HEADER 기준으로 계속합니다.\n")
                else:
                    print("Arduino 통합 header 수신 완료")
                header_seen = True
                continue

            values = [v.strip() for v in line.split(",")]

            # 헤더를 못 봤더라도 데이터 행이면 저장 시작
            if not header_seen and START_WITH_BUILTIN_HEADER_IF_DATA_COMES:
                if looks_like_data_row(values):
                    header_seen = True
                    print("Arduino header를 놓친 것으로 판단하여 내장 HEADER로 저장을 시작합니다.")
                else:
                    print("헤더 대기 중:", line)
                    continue

            if len(values) != len(HEADER):
                bad_count += 1
                print(f"컬럼 수 불일치({len(values)}개 / 기대 {len(HEADER)}개): {line}")
                continue

            ppg_mode = values[1]
            gsr_mode = values[32]
            analysis_ppg_mode = normalize_ppg_mode(ppg_mode)
            analysis_gsr_mode = normalize_gsr_mode(gsr_mode)

            row = values + [f"{pc_elapsed_sec:.2f}", analysis_ppg_mode, analysis_gsr_mode]

            writer.writerow(row)
            csv_file.flush()
            rows.append(row)

            # 보기 좋게 주요 값만 터미널 출력
            time_ms = values[0]
            ir_raw = values[2]
            mean_hr = values[6]
            sdnn = values[7]
            rmssd = values[8]
            final_ppg = values[27]
            final_ppg_level = values[28]

            raw_gsr = values[34]
            scl_mean_30s = values[40]
            scr_amp_30s = values[41]
            ns_scr_freq = values[42]
            final_gsr = values[49]
            gsr_ready = values[50]

            integrated_score = values[53]
            integrated_level = values[54]

            print(
                f"[{int(pc_elapsed_sec):4d}s | t={time_ms:>7s} ms] "
                f"PPG={analysis_ppg_mode:10s} "
                f"GSR={analysis_gsr_mode:11s} "
                f"IR={ir_raw:>6s} "
                f"HR={mean_hr:>6s} SDNN={sdnn:>6s} RMSSD={rmssd:>6s} "
                f"PPGscore={final_ppg or '-':>6s} {final_ppg_level or '-':>6s} "
                f"GSRraw={raw_gsr:>4s} SCL30={scl_mean_30s or '-':>6s} "
                f"SCRamp={scr_amp_30s or '-':>6s} NSfreq={ns_scr_freq or '-':>6s} "
                f"GSRscore={final_gsr or '-':>6s} ready={gsr_ready:>1s} "
                f"TOTAL={integrated_score or '-':>6s} {integrated_level or '-':>6s} "
                f"rows={len(rows)}"
            )

except KeyboardInterrupt:
    print("\n사용자 중단")

finally:
    ser.close()
    print("\nSerial 연결 종료")
    print(f"총 저장 row: {len(rows)}")
    print(f"빈 수신 횟수: {empty_count}")
    print(f"컬럼 불일치 횟수: {bad_count}")


# ==============================
# 8. 저장 결과 확인
# ==============================
if not OUTPUT_CSV.exists():
    raise FileNotFoundError("CSV 파일이 생성되지 않았습니다.")

df = pd.read_csv(OUTPUT_CSV)

if df.empty:
    print("\nCSV 파일은 생성되었지만 저장된 데이터가 없습니다.")
    raise ValueError("CSV 파일은 생성되었지만 저장된 데이터가 없습니다.")

print("\n====================================")
print("CSV 저장 완료")
print(f"파일명: {OUTPUT_CSV}")
print(f"row 수: {len(df)}")
print("컬럼:")
print(df.columns.tolist())
print("====================================")

print("\nPPG mode별 개수:")
print(df["analysis_ppg_mode"].value_counts(dropna=False))

print("\nGSR mode별 개수:")
print(df["analysis_gsr_mode"].value_counts(dropna=False))

print("\n통합 스트레스 level별 개수:")
print(df["integrated_stress_level"].replace("", pd.NA).value_counts(dropna=False))


# ==============================
# 9. 숫자형 변환
# ==============================
non_numeric_cols = {
    "ppg_mode",
    "final_ppg_stress_level",
    "gsr_mode",
    "integrated_stress_level",
    "analysis_ppg_mode",
    "analysis_gsr_mode",
}

numeric_cols = [c for c in FULL_HEADER if c not in non_numeric_cols]

for col in numeric_cols:
    df[col] = pd.to_numeric(df[col], errors="coerce")

df["time_sec"] = df["time_ms"] / 1000.0

# 유효하지 않은 IBI 더미값(0 이하) → NaN으로 처리해 그래프 노이즈 방지
if "valid_ibi_ms" in df.columns:
    df.loc[df["valid_ibi_ms"] <= INVALID_IBI_THRESHOLD, "valid_ibi_ms"] = pd.NA

# 숫자 변환까지 반영해서 다시 저장
df.to_csv(OUTPUT_CSV, index=False, encoding="utf-8-sig")


# ==============================
# 10. mode별 요약
# ==============================
print("\n====================================")
print("분석용 mode별 주요 지표 요약")
print("====================================")

summary_cols = [
    "mean_hr",
    "sdnn",
    "rmssd",
    "valid_ibi_ms",
    "final_ppg_stress_score",
    "scl_mean_30s",
    "scr_amplitude_mean_30s",
    "ns_scr_frequency_per_min",
    "final_gsr_stress_score",
    "integrated_stress_score",
]

for mode in ["STABILIZING", "BASELINE", "MONITORING", "NO_FINGER"]:
    mode_df = df[df["analysis_ppg_mode"] == mode].copy()
    if mode_df.empty:
        continue

    print(f"\n[PPG {mode}]")
    for col in summary_cols:
        if col not in mode_df.columns:
            continue
        valid_series = mode_df[col].dropna()
        if len(valid_series) > 0:
            print(
                f"{col}: "
                f"mean={valid_series.mean():.2f}, "
                f"min={valid_series.min():.2f}, "
                f"max={valid_series.max():.2f}"
            )
        else:
            print(f"{col}: valid data 없음")


# ==============================
# 11. 그래프 저장
# ==============================
if SAVE_GRAPHS:
    graph_dir = OUTPUT_DIR / f"graphs_{timestamp}"
    graph_dir.mkdir(exist_ok=True)

    def add_mode_lines(ax, df: pd.DataFrame, mode_col: str) -> None:
        """mode가 바뀌는 지점에 세로선 표시"""
        if mode_col not in df.columns:
            return

        transitions = df[df[mode_col] != df[mode_col].shift()]
        _, y_max = ax.get_ylim()

        for _, row in transitions.iterrows():
            x = row["pc_elapsed_sec"]
            label = row[mode_col]
            ax.axvline(x=x, linestyle="--", linewidth=1)
            ax.text(x, y_max, str(label), rotation=90, va="top", fontsize=8)

    x_col = "pc_elapsed_sec"

    plot_targets = [
        ("ir_raw", "IR Raw", "IR Raw", "01_ir_raw.png", "analysis_ppg_mode"),
        ("filtered_ppg", "Filtered PPG", "Filtered PPG", "02_filtered_ppg.png", "analysis_ppg_mode"),
        ("mean_hr", "Mean HR over Time", "Mean HR (bpm)", "03_mean_hr.png", "analysis_ppg_mode"),
        ("sdnn", "SDNN over Time", "SDNN (ms)", "04_sdnn.png", "analysis_ppg_mode"),
        ("rmssd", "RMSSD over Time", "RMSSD (ms)", "05_rmssd.png", "analysis_ppg_mode"),
        ("valid_ibi_ms", "Valid IBI over Time", "IBI (ms)", "06_valid_ibi.png", "analysis_ppg_mode"),
        ("raw_gsr", "GSR Raw over Time", "GSR ADC", "07_raw_gsr.png", "analysis_gsr_mode"),
        ("scl_mean_30s", "GSR SCL Mean 30s", "SCL Mean", "08_scl_mean_30s.png", "analysis_gsr_mode"),
        ("scr_amplitude_mean_30s", "SCR Amplitude Mean 30s", "SCR Amplitude", "09_scr_amplitude_30s.png", "analysis_gsr_mode"),
        ("ns_scr_frequency_per_min", "NS-SCR Frequency per Min", "Frequency", "10_ns_scr_frequency.png", "analysis_gsr_mode"),
        ("final_ppg_stress_score", "Final PPG Stress Score", "Score", "11_final_ppg_score.png", "analysis_ppg_mode"),
        ("final_gsr_stress_score", "Final GSR Stress Score", "Score", "12_final_gsr_score.png", "analysis_gsr_mode"),
        ("integrated_stress_score", "Integrated Stress Score", "Score", "13_integrated_score.png", "analysis_ppg_mode"),
    ]

    for col, title, ylabel, filename, mode_col in plot_targets:
        if col not in df.columns:
            continue

        valid_df = df[[x_col, col, mode_col]].dropna(subset=[x_col, col])
        if valid_df.empty:
            continue

        fig, ax = plt.subplots(figsize=(14, 4))

        if col == "valid_ibi_ms":
            ax.plot(valid_df[x_col], valid_df[col], marker="o", markersize=3)
        else:
            ax.plot(valid_df[x_col], valid_df[col])

        ax.set_title(title)
        ax.set_xlabel("PC elapsed time (s)")
        ax.set_ylabel(ylabel)
        ax.grid(True)

        add_mode_lines(ax, df, mode_col)

        fig.tight_layout()
        fig.savefig(graph_dir / filename, dpi=200)
        plt.close(fig)

    print(f"\n그래프 저장 완료: {graph_dir}")
