import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError:
    print("pyserial이 설치되어 있지 않습니다. 아래 명령으로 설치하세요:")
    print("  pip install pyserial")
    raise

# ============================================================
# PPG/GSR Integrated Serial CSV Logger for v7 / 수정4_ppg_400hz.cpp
# - Enter 입력 시 MCU에 START_COMMAND 전송
# - MCU가 CSV header를 보내면 그 header를 그대로 사용
# - MCU가 header 없이 data row부터 보내도 현재 펌웨어용 DEFAULT_HEADER로 저장
# - 모든 수신 CSV column을 그대로 저장
# - 별도 .log 파일에 MCU 메시지/원본 line도 저장
# ============================================================

DEFAULT_PORT = "COM5"
DEFAULT_BAUD_RATE = 115200
SAVE_DIR = "integrated_serial_csv_logs"
FILE_PREFIX = "ppg_gsr_integrated_v8"

START_COMMAND = "S"
STOP_COMMAND = "X"
STOP_KEY = "x"
SERIAL_TIMEOUT_SEC = 1
STATUS_EVERY_ROWS = 10

# 수정4_ppg_400hz.cpp의 Serial.println header와 동일하게 맞춤
DEFAULT_HEADER = [
    "timestamp",
    "mode",
    "raw_gsr_adc",
    "gsr_relative_conductance",
    "ir_raw",
    "filtered_ppg",
    "ppg_mean_hr",
    "ppg_sdnn",
    "ppg_rmssd",
    "gsr_scl_relative",
    "gsr_scr_amp_relative",
    "gsr_scr_freq",
    "ppg_sdnn_z",
    "ppg_rmssd_z",
    "gsr_scl_z",
    "gsr_scr_amp_z",
    "gsr_scr_freq_z",
    "final_ppg_stress_z",
    "final_gsr_stress_z",
    "valid_feature_count",
    "integrated_stress_z",
    "integrated_state",
]


def make_output_paths(save_dir: str, prefix: str):
    Path(save_dir).mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = Path(save_dir) / f"{prefix}_{stamp}"
    return base.with_suffix(".csv"), base.with_suffix(".log"), base.with_suffix(".json")


def wait_for_enter():
    print("====================================")
    print("PPG/GSR Integrated Serial CSV Logger")
    print("====================================")
    print("MCU 코드에서 ENABLE_SERIAL_CSV = true 인지 확인하세요.")
    print("Arduino Serial Monitor가 열려 있으면 포트 충돌이 납니다. 닫아 주세요.")
    print("측정을 시작하려면 Enter 키를 누르세요.")
    print("Windows에서는 측정 중 x 키, 그 외 환경은 Ctrl+C로 종료합니다.")
    print("====================================")
    input("Enter를 누르면 측정을 시작합니다...")


def windows_stop_key_pressed():
    if os.name != "nt":
        return False
    try:
        import msvcrt
        if msvcrt.kbhit():
            key = msvcrt.getch().decode("utf-8", errors="ignore").lower()
            return key == STOP_KEY.lower()
    except Exception:
        return False
    return False


def is_header(parts):
    lowered = [p.strip().lower() for p in parts]
    return "timestamp" in lowered and "mode" in lowered


def looks_like_data_row(parts):
    if len(parts) != len(DEFAULT_HEADER):
        return False
    try:
        float(parts[0])
        return True
    except Exception:
        return False


def row_to_dict(header, row):
    return {name: row[i].strip() if i < len(row) else "" for i, name in enumerate(header)}


def build_status_message(header, row, row_count, elapsed_real_sec):
    data = row_to_dict(header, row)
    return (
        f"[측정중] rows={row_count} | "
        f"MCU time={data.get('timestamp', '-')} ms | "
        f"mode={data.get('mode', '-')} | "
        f"raw_gsr_adc={data.get('raw_gsr_adc', '-')} | "
        f"gsr_rel={data.get('gsr_relative_conductance', '-')} | "
        f"ir_raw={data.get('ir_raw', '-')} | "
        f"filtered_ppg={data.get('filtered_ppg', '-')} | "
        f"sdnn={data.get('ppg_sdnn', '-')} | "
        f"rmssd={data.get('ppg_rmssd', '-')} | "
        f"scl_rel={data.get('gsr_scl_relative', '-')} | "
        f"scr_amp_rel={data.get('gsr_scr_amp_relative', '-')} | "
        f"ppg_z={data.get('final_ppg_stress_z', '-')} | "
        f"gsr_z={data.get('final_gsr_stress_z', '-')} | "
        f"integrated_z={data.get('integrated_stress_z', '-')} | "
        f"state={data.get('integrated_state', '-')} | "
        f"real_elapsed={elapsed_real_sec:.1f}s"
    )


def save_metadata(meta_path, args, csv_path, log_path):
    metadata = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "port": args.port,
        "baud_rate": args.baud,
        "csv_path": str(csv_path),
        "log_path": str(log_path),
        "start_command": START_COMMAND,
        "stop_command": STOP_COMMAND,
        "default_header": DEFAULT_HEADER,
        "notes": [
            "CSV에는 MCU가 Serial로 전송한 모든 column을 그대로 저장합니다.",
            "현재 펌웨어는 1초마다 요약 row를 출력하므로, 100Hz PPG 전체 raw sample을 offline 재계산하려면 펌웨어에서 raw sample 출력 모드가 추가로 필요합니다.",
        ],
    }
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(metadata, f, ensure_ascii=False, indent=2)


def main():
    parser = argparse.ArgumentParser(description="PPG/GSR Integrated Serial CSV Logger")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Arduino serial port, 예: COM5")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD_RATE, help="Baud rate")
    parser.add_argument("--save-dir", default=SAVE_DIR, help="CSV 저장 폴더")
    parser.add_argument("--prefix", default=FILE_PREFIX, help="저장 파일 prefix")
    args = parser.parse_args()

    csv_path, log_path, meta_path = make_output_paths(args.save_dir, args.prefix)
    save_metadata(meta_path, args, csv_path, log_path)

    print(f"PORT: {args.port}")
    print(f"BAUD_RATE: {args.baud}")
    print(f"CSV 저장 예정 파일: {csv_path}")
    print(f"원본 로그 저장 예정 파일: {log_path}")
    print(f"메타데이터 저장 파일: {meta_path}")
    wait_for_enter()

    row_count = 0
    skip_count = 0
    blank_count = 0
    mismatch_count = 0
    header = None
    start_real_time = None
    ser = None

    try:
        with serial.Serial(args.port, args.baud, timeout=SERIAL_TIMEOUT_SEC) as ser, \
             open(csv_path, "w", newline="", encoding="utf-8-sig") as csv_file, \
             open(log_path, "w", encoding="utf-8") as log_file:

            writer = csv.writer(csv_file)

            print("\nSerial 연결 완료")
            time.sleep(0.5)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.write((START_COMMAND + "\n").encode("utf-8"))
            ser.flush()
            print(f"MCU에 측정 시작 명령 전송 완료: {START_COMMAND}")
            print("CSV 수신 대기 중...")
            print("====================================")

            while True:
                if windows_stop_key_pressed():
                    print(f"\n[{STOP_KEY}] 입력 확인 → 측정 종료")
                    break

                raw = ser.readline()
                recv_time = datetime.now().isoformat(timespec="milliseconds")

                if not raw:
                    blank_count += 1
                    if blank_count % 10 == 0 and row_count == 0:
                        print("[대기중] 아직 CSV 데이터를 받지 못했습니다.")
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    blank_count += 1
                    continue

                log_file.write(f"{recv_time}\t{line}\n")

                if "," not in line:
                    print(f"[MCU] {line}")
                    skip_count += 1
                    continue

                parts = [p.strip() for p in line.split(",")]

                if header is None and is_header(parts):
                    header = parts
                    writer.writerow(header)
                    csv_file.flush()
                    start_real_time = time.time()
                    print("CSV header 저장 완료")
                    print(",".join(header))
                    print("====================================")
                    continue

                if header is None and looks_like_data_row(parts):
                    header = DEFAULT_HEADER
                    writer.writerow(header)
                    writer.writerow(parts)
                    row_count += 1
                    csv_file.flush()
                    start_real_time = time.time()
                    print("[INFO] MCU에서 header 없이 데이터가 먼저 수신되었습니다.")
                    print("[INFO] 기본 header를 사용해 저장을 시작합니다.")
                    print(",".join(header))
                    print(build_status_message(header, parts, row_count, 0.0))
                    print("====================================")
                    continue

                if header is not None and is_header(parts):
                    print("[INFO] CSV header가 다시 수신되었습니다. 저장은 계속 진행합니다.")
                    continue

                if header is None:
                    print(f"[SKIP: header 전 알 수 없는 데이터] {line}")
                    skip_count += 1
                    continue

                if len(parts) != len(header):
                    mismatch_count += 1
                    print(f"[SKIP: 컬럼 수 불일치] expected={len(header)}, got={len(parts)} | {line}")
                    continue

                writer.writerow(parts)
                row_count += 1

                if row_count % STATUS_EVERY_ROWS == 0:
                    csv_file.flush()
                    log_file.flush()
                    elapsed = time.time() - start_real_time if start_real_time else 0.0
                    print(build_status_message(header, parts, row_count, elapsed))

    except KeyboardInterrupt:
        print("\n사용자 중단")

    except serial.SerialException as e:
        print("\nSerial 오류 발생")
        print(e)
        print("포트 번호가 맞는지, Arduino Serial Monitor가 열려 있지 않은지 확인하세요.")
        sys.exit(1)

    finally:
        try:
            if ser is not None and ser.is_open:
                ser.write((STOP_COMMAND + "\n").encode("utf-8"))
                ser.flush()
                print(f"MCU에 측정 정지 명령 전송 완료: {STOP_COMMAND}")
        except Exception:
            pass

        print("\n====================================")
        print("CSV 저장 종료")
        print(f"저장 파일: {csv_path}")
        print(f"원본 로그: {log_path}")
        print(f"저장 row 수: {row_count}")
        print(f"건너뛴 row 수: {skip_count}")
        print(f"빈 수신 횟수: {blank_count}")
        print(f"컬럼 불일치 횟수: {mismatch_count}")
        print("====================================")


if __name__ == "__main__":
    main()
