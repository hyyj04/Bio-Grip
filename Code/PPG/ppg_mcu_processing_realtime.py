import serial
import csv
import time
from datetime import datetime
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt


# ==============================
# 1. 사용자 설정
# ==============================
SERIAL_PORT = "COM3"
BAUD_RATE = 115200

# None이면 사용자가 Ctrl+C로 직접 종료
RECORD_SECONDS = None

OUTPUT_DIR = Path("ppg_realtime_results")
OUTPUT_DIR.mkdir(exist_ok=True)

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
OUTPUT_CSV = OUTPUT_DIR / f"ppg_mcu_realtime_processing_{timestamp}.csv"

SAVE_GRAPHS = True

# 아두이노에서 유효하지 않은 IBI를 나타내는 더미값 (0 이하는 무효 처리)
INVALID_IBI_THRESHOLD = 0


# ==============================
# 2. 아두이노 출력 컬럼
# ==============================
HEADER = [
    "time_ms",
    "mode",
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
    "ibi_count",
    "baseline_count",
    "no_finger_count",
]


# ==============================
# 3. 실행 정보 출력
# ==============================
print("====================================")
print("PPG Serial CSV 저장 시작")
print(f"PORT      : {SERIAL_PORT}")
print(f"BAUD RATE : {BAUD_RATE}")
print(f"OUTPUT    : {OUTPUT_CSV}")
print("MODE      : STABILIZING / BASELINE / MONITORING")
print("종료 방법 : Ctrl + C")
print("====================================")


# ==============================
# 4. CSV 파일 먼저 생성
# ==============================
with open(OUTPUT_CSV, "w", newline="", encoding="utf-8-sig") as f:
    writer = csv.writer(f)
    writer.writerow(HEADER + ["pc_elapsed_sec", "analysis_mode"])

print("CSV 파일 생성 완료")
print(f"저장 경로: {OUTPUT_CSV}")


# ==============================
# 5. Serial 연결
# ==============================
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except serial.SerialException as e:
    print("\nSerial 포트를 열 수 없습니다.")
    print("확인:")
    print("1. 아두이노 IDE 시리얼 모니터가 닫혀 있는지")
    print("2. COM 포트 번호가 맞는지")
    print("3. USB 케이블이 연결되어 있는지")
    raise e

time.sleep(2)

print("\nSerial 연결 완료")
print("VSC 터미널에서 실시간 수신값을 표시하면서 CSV 저장을 시작합니다.")
print("측정을 중단하려면 Ctrl + C를 누르세요.\n")


rows = []
empty_count = 0
bad_count = 0
start_time = time.time()


# ==============================
# 6. mode 정리 함수
# ==============================
def normalize_mode(arduino_mode: str) -> str:
    """
    보고서/분석용 mode를 STABILIZING / BASELINE / MONITORING으로 단순화한다.
    아두이노에서 CALIBRATION으로 넘어오면 BASELINE으로 저장한다.
    NO_FINGER는 별도 상태로 보존한다.
    """
    mode_map = {
        "STABILIZING": "STABILIZING",
        "CALIBRATION": "BASELINE",
        "MONITORING":  "MONITORING",
        "NO_FINGER":   "NO_FINGER",
    }
    return mode_map.get(arduino_mode, arduino_mode)


# ==============================
# 7. Serial 데이터 수신 + 터미널 출력 + CSV 저장
# [FIX] CSV 파일을 루프 바깥에서 한 번만 열어 성능 개선
# ==============================
try:
    with open(OUTPUT_CSV, "a", newline="", encoding="utf-8-sig") as csv_file:
        writer = csv.writer(csv_file)

        while True:
            raw = ser.readline()

            # [FIX] readline() 블로킹 이후에 elapsed 측정 → 실제 수신 시각 반영
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

            if not line:
                empty_count += 1
                if empty_count % 5 == 0:
                    print(f"{int(pc_elapsed_sec)} sec / 빈 줄 수신 중... empty_count={empty_count}")
                continue

            if line.startswith("time_ms"):
                print("Arduino header 수신:", line)
                continue

            if line.startswith("ERROR"):
                print("Arduino ERROR:", line)
                continue

            values = line.split(",")

            if len(values) != len(HEADER):
                bad_count += 1
                print(f"컬럼 수 불일치({len(values)}개 / 기대 {len(HEADER)}개): {line}")
                continue

            arduino_mode = values[1]
            analysis_mode = normalize_mode(arduino_mode)

            row = values + [f"{pc_elapsed_sec:.2f}", analysis_mode]

            writer.writerow(row)
            csv_file.flush()  # 강제 flush로 데이터 유실 방지

            rows.append(row)

            time_ms = values[0]
            ir_raw = values[2]
            filtered_ppg = values[3]
            valid_ibi_ms = values[5]
            mean_hr = values[6]
            sdnn = values[7]
            rmssd = values[8]
            ibi_count = values[15]
            baseline_count = values[16]
            no_finger_count = values[17]

            print(
                f"[{int(pc_elapsed_sec):4d}s | {analysis_mode:11s}] "
                f"rawMode={arduino_mode:11s} "
                f"IR={ir_raw:>6s} "
                f"PPG={filtered_ppg:>10s} "
                f"IBI={valid_ibi_ms:>7s} "
                f"HR={mean_hr:>6s} "
                f"SDNN={sdnn:>6s} "
                f"RMSSD={rmssd:>6s} "
                f"ibiN={ibi_count:>3s} "
                f"baseN={baseline_count:>4s} "
                f"noF={no_finger_count:>4s} "
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

print("\nArduino mode별 개수:")
print(df["mode"].value_counts())

print("\n분석용 mode별 개수:")
print(df["analysis_mode"].value_counts())


# ==============================
# 9. 숫자형 변환
# ==============================
numeric_cols = [
    "time_ms",
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
    "ibi_count",
    "baseline_count",
    "no_finger_count",
    "pc_elapsed_sec",
]

for col in numeric_cols:
    df[col] = pd.to_numeric(df[col], errors="coerce")

df["time_sec"] = df["time_ms"] / 1000.0

# [FIX] 유효하지 않은 IBI 더미값(0 이하) → NaN으로 처리해 그래프 노이즈 방지
df.loc[df["valid_ibi_ms"] <= INVALID_IBI_THRESHOLD, "valid_ibi_ms"] = pd.NA

df.to_csv(OUTPUT_CSV, index=False, encoding="utf-8-sig")


# ==============================
# 10. mode별 요약
# ==============================
print("\n====================================")
print("분석용 mode별 주요 지표 요약")
print("====================================")

for mode in ["STABILIZING", "BASELINE", "MONITORING", "NO_FINGER"]:
    mode_df = df[df["analysis_mode"] == mode].copy()

    if mode_df.empty:
        continue

    print(f"\n[{mode}]")
    for col in ["mean_hr", "sdnn", "rmssd", "valid_ibi_ms"]:
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

    # [FIX] df를 인자로 받도록 수정 (전역 참조 제거)
    # [FIX] shift()로 전환점만 추출해 성능 개선
    def add_mode_lines(ax, df: pd.DataFrame) -> None:
        """analysis_mode가 바뀌는 지점에 세로선 표시"""
        if "analysis_mode" not in df.columns:
            return

        transitions = df[df["analysis_mode"] != df["analysis_mode"].shift()]
        y_min, y_max = ax.get_ylim()

        for _, row in transitions.iterrows():
            x = row["pc_elapsed_sec"]
            ax.axvline(x=x, linestyle="--", linewidth=1, color="gray")
            ax.text(x, y_max, row["analysis_mode"], rotation=90, va="top", fontsize=8)

    x_col = "pc_elapsed_sec"

    plot_targets = [
        ("ir_raw",       "IR Raw",            "IR Raw",        "01_ir_raw.png"),
        ("filtered_ppg", "Filtered PPG",       "Filtered PPG",  "02_filtered_ppg.png"),
        ("mean_hr",      "Mean HR over Time",  "Mean HR (bpm)", "03_mean_hr.png"),
        ("sdnn",         "SDNN over Time",     "SDNN (ms)",     "04_sdnn.png"),
        ("rmssd",        "RMSSD over Time",    "RMSSD (ms)",    "05_rmssd.png"),
        ("valid_ibi_ms", "Valid IBI over Time","IBI (ms)",      "06_valid_ibi.png"),
    ]

    for col, title, ylabel, filename in plot_targets:
        fig, ax = plt.subplots(figsize=(14, 4))

        if col == "valid_ibi_ms":
            ax.plot(df[x_col], df[col], marker="o", markersize=3)
        else:
            ax.plot(df[x_col], df[col])

        ax.set_title(title)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(ylabel)
        ax.grid(True)

        add_mode_lines(ax, df)  # [FIX] ax와 df를 명시적으로 전달

        fig.tight_layout()
        fig.savefig(graph_dir / filename, dpi=200)
        plt.close(fig)

    print(f"\n그래프 저장 완료: {graph_dir}")