"""
Arduino Serial CSV Logger for integrated PPG + GSR stress-score code

Usage:
1) pip install pyserial
2) Set COM_PORT below, or leave as "AUTO" to select from detected ports.
3) Run: python save_integrated_csv.py
4) Stop with Ctrl+C. The CSV file is saved continuously.
"""

import csv
import os
import sys
import time
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pyserial이 설치되어 있지 않습니다.")
    print("터미널에서 먼저 실행하세요: pip install pyserial")
    sys.exit(1)

# =========================
# User settings
# =========================
COM_PORT = "AUTO"       # 예: Windows "COM3", Mac/Linux "/dev/ttyACM0". 자동 선택은 "AUTO"
BAUD_RATE = 115200      # Arduino 코드의 Serial.begin(115200)과 동일해야 함
SAVE_DIR = "csv_logs"   # CSV 저장 폴더
FILE_PREFIX = "ppg_gsr_integrated"
PRINT_EVERY_N_ROWS = 1  # 화면에 몇 줄마다 출력할지. 너무 빠르면 5 또는 10으로 변경

# Arduino 통합 코드의 CSV 헤더 시작 문자열
HEADER_START = "time_ms,ppg_mode,ir_raw"


def choose_serial_port() -> str:
    """Return selected serial port. If COM_PORT is not AUTO, return it directly."""
    if COM_PORT.upper() != "AUTO":
        return COM_PORT

    ports = list(list_ports.comports())
    if not ports:
        print("연결된 시리얼 포트를 찾지 못했습니다.")
        print("Arduino가 연결되어 있는지, Arduino IDE의 Serial Monitor가 닫혀 있는지 확인하세요.")
        sys.exit(1)

    print("감지된 시리얼 포트:")
    for idx, port in enumerate(ports, start=1):
        print(f"  {idx}. {port.device} - {port.description}")

    if len(ports) == 1:
        selected = ports[0].device
        print(f"\n포트가 1개라서 자동 선택합니다: {selected}")
        return selected

    while True:
        choice = input("사용할 포트 번호를 입력하세요: ").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            return ports[int(choice) - 1].device
        print("올바른 번호를 입력하세요.")


def make_output_path() -> str:
    os.makedirs(SAVE_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(SAVE_DIR, f"{FILE_PREFIX}_{timestamp}.csv")


def parse_csv_line(line: str):
    """Parse one CSV line safely."""
    return next(csv.reader([line]))


def main():
    port = choose_serial_port()
    output_path = make_output_path()

    print("\n시리얼 연결 중...")
    print(f"포트: {port}")
    print(f"Baud rate: {BAUD_RATE}")
    print(f"저장 파일: {output_path}")
    print("중지하려면 Ctrl+C를 누르세요.\n")

    row_count = 0
    header = None

    try:
        with serial.Serial(port, BAUD_RATE, timeout=1) as ser, open(
            output_path, "w", newline="", encoding="utf-8-sig"
        ) as f:
            writer = csv.writer(f)

            # Arduino reset 후 setup 출력 대기
            time.sleep(2)
            ser.reset_input_buffer()

            while True:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                # 센서 오류 메시지는 CSV 데이터가 아니므로 화면에만 표시
                if line.startswith("ERROR"):
                    print(f"Arduino error: {line}")
                    continue

                # 헤더 찾기
                if line.startswith(HEADER_START):
                    header = parse_csv_line(line)
                    writer.writerow(header)
                    f.flush()
                    print("CSV 헤더 감지 완료. 저장을 시작합니다.")
                    print(line)
                    continue

                # 헤더가 나오기 전 데이터/잡음은 저장하지 않음
                if header is None:
                    print(f"헤더 대기 중: {line}")
                    continue

                # 데이터 행 저장
                row = parse_csv_line(line)

                # 컬럼 수가 헤더와 다르면 저장은 하되 경고 출력
                if len(row) != len(header):
                    print(f"경고: 컬럼 수 불일치 header={len(header)}, row={len(row)}")
                    print(line)

                writer.writerow(row)
                row_count += 1

                if row_count % PRINT_EVERY_N_ROWS == 0:
                    print(f"[{row_count:05d}] {line}")

                # 갑자기 종료되어도 파일 손실을 줄이기 위해 주기적으로 flush
                if row_count % 10 == 0:
                    f.flush()

    except serial.SerialException as e:
        print("\n시리얼 포트 오류가 발생했습니다.")
        print(e)
        print("Arduino IDE의 Serial Monitor/Plotter가 열려 있으면 닫고 다시 실행하세요.")
        sys.exit(1)

    except KeyboardInterrupt:
        print("\n저장을 중지했습니다.")
        print(f"저장된 파일: {output_path}")
        print(f"저장된 데이터 행 수: {row_count}")


if __name__ == "__main__":
    main()
