import serial
import csv
import os
from datetime import datetime

# =====================
# 설정
# =====================
PORT = "COM5"          # 본인 아두이노 포트로 변경
BAUD_RATE = 115200

SAVE_DIR = "integrated_serial_csv_logs"
FILE_PREFIX = "ppg_gsr_integrated_serial"

os.makedirs(SAVE_DIR, exist_ok=True)
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
output_path = os.path.join(SAVE_DIR, f"{FILE_PREFIX}_{timestamp}.csv")

print("====================================")
print("PPG/GSR Integrated Serial CSV Logger")
print("====================================")
print(f"PORT: {PORT}")
print(f"BAUD_RATE: {BAUD_RATE}")
print(f"저장 파일: {output_path}")
print("====================================")

row_count = 0
skip_count = 0
header_saved = False

try:
    with serial.Serial(PORT, BAUD_RATE, timeout=1) as ser, open(
        output_path, "w", newline="", encoding="utf-8-sig"
    ) as f:

        writer = None

        print("Serial 연결 완료")
        print("MCU가 READY 상태이면 s 입력 후 Enter를 눌러 측정을 시작하세요.")
        print("종료하려면 Ctrl+C")
        print("====================================")

        # MCU에 측정 시작 명령 전송
        input("측정을 시작하려면 Enter를 누르세요...")
        ser.write(b"s\n")
        print("측정 시작 명령 전송 완료")

        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()

            if not line:
                continue

            # CSV가 아닌 이벤트/로그 문구는 건너뜀
            # ENABLE_SERIAL_EVENTS=false라면 거의 안 나와야 하지만 안전장치로 둠
            if "," not in line:
                print(f"[SKIP] {line}")
                skip_count += 1
                continue

            parts = line.split(",")

            # 첫 CSV header 저장
            if not header_saved:
                writer = csv.writer(f)
                writer.writerow(parts)
                header_saved = True
                print("CSV header 저장 완료")
                print(parts)
                continue

            # 데이터 row 저장
            writer.writerow(parts)
            row_count += 1

            if row_count % 10 == 0:
                f.flush()
                print(f"저장 row 수: {row_count}")

except KeyboardInterrupt:
    print("\n사용자 중단")

except serial.SerialException as e:
    print("\nSerial 오류 발생")
    print(e)

finally:
    print("\n====================================")
    print("CSV 저장 종료")
    print(f"저장 파일: {output_path}")
    print(f"저장 row 수: {row_count}")
    print(f"건너뛴 row 수: {skip_count}")
    print("====================================")