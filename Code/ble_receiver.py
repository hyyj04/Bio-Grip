import asyncio
import csv
import struct
from datetime import datetime
from bleak import BleakClient, BleakScanner

DEVICE_NAME        = "BioGrip Analyzer"
FEATURE_CHAR_UUID  = "19b10003-e8f2-537e-4f6c-d104768a1214"
RAW_DATA_CHAR_UUID = "19b10002-e8f2-537e-4f6c-d104768a1214"
CONTROL_CHAR_UUID  = "19b10004-e8f2-537e-4f6c-d104768a1214"

CSV_FILENAME = f"biogripdata_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

HEADERS = [
    "Timestamp", "Type", "Raw_Value",
    "PPG_Mean_HR", "PPG_SDNN", "PPG_RMSSD",
    "GSR_SCL_Mean", "GSR_SCR_Amp", "GSR_NSSCR_Freq"
]

def write_row(row: list):
    with open(CSV_FILENAME, mode='a', newline='') as f:
        csv.writer(f).writerow(row)

# ── Raw 수신 핸들러 ──
def raw_handler(sender, data):
    if len(data) == 5:
        sensor_type = chr(data[0])                       # 'P' or 'G'
        value       = struct.unpack('<I', data[1:5])[0]  # uint32_t
        timestamp   = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

        # Type, Raw_Value만 채우고 Feature 컬럼은 빈칸
        write_row([timestamp, sensor_type, value, "", "", "", "", "", ""])
    else:
        print(f"경고: Raw 데이터 길이 오류 ({len(data)} bytes)")

# ── Feature 수신 핸들러 ──
def feature_handler(sender, data):
    if len(data) == 24:
        hr, sdnn, rmssd, scl, scr, nsscr = struct.unpack('<ffffff', data)
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

        print(f"[{timestamp}] Feature: HR={hr:.2f} SDNN={sdnn:.2f} RMSSD={rmssd:.2f} "
              f"SCL={scl:.2f} SCR={scr:.2f} NSSCR={nsscr:.2f}")

        # Raw 컬럼은 빈칸, Feature 컬럼만 채움
        write_row([timestamp, "FEATURE", "", hr, sdnn, rmssd, scl, scr, nsscr])
    else:
        print(f"경고: Feature 데이터 길이 오류 ({len(data)} bytes)")

async def main():
    print(f"'{DEVICE_NAME}' 장치를 찾는 중입니다...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)

    if not device:
        print("장치를 찾을 수 없습니다.")
        return

    print(f"장치 발견: {device.address}")

    # CSV 헤더 초기화
    with open(CSV_FILENAME, mode='w', newline='') as f:
        csv.writer(f).writerow(HEADERS)
    print(f"CSV 파일 생성: {CSV_FILENAME}\n")

    async with BleakClient(device) as client:
        print("연결 성공!")

        await client.start_notify(RAW_DATA_CHAR_UUID, raw_handler)
        await client.start_notify(FEATURE_CHAR_UUID,  feature_handler)
        print("Raw + Feature 수신 대기 모드 온!")

        try:
            print("아두이노에 시뮬레이션 시작 명령(1)을 전송합니다...")
            await client.write_gatt_char(CONTROL_CHAR_UUID, bytearray([1]), response=True)
        except Exception as e:
            print(f"명령 전송 실패: {e}")

        print(f"데이터 기록 중... (종료: Ctrl+C)\n")

        try:
            while True:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            pass
        finally:
            print("\n수신 종료 및 연결 해제.")
            await client.stop_notify(RAW_DATA_CHAR_UUID)
            await client.stop_notify(FEATURE_CHAR_UUID)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n프로그램 종료.")