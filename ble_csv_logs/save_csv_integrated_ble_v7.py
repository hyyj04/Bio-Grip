import argparse
import asyncio
import csv
import json
import os
import signal
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    print("bleak이 설치되어 있지 않습니다. 아래 명령으로 설치하세요:")
    print("  pip install bleak")
    raise

# ============================================================
# PPG/GSR Integrated BLE CSV Logger for ppg_gsr_integrated_v7.ino
# - BLE peripheral 이름: BioGrip_Feature
# - controlPointChar에 1을 쓰면 측정 시작, 2를 쓰면 측정 정지
# - featureDataChar notify로 들어오는 packed struct를 CSV로 저장
# - Serial CSV와 달리 BLE 펌웨어가 보내는 항목만 저장 가능
# ============================================================

DEVICE_NAME = "BioGrip_Feature"
SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"
FEATURE_CHAR_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
CONTROL_CHAR_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"

SAVE_DIR = "integrated_ble_csv_logs"
FILE_PREFIX = "ppg_gsr_integrated_ble_v7"
STOP_KEY = "x"
SCAN_TIMEOUT_SEC = 10.0
STATUS_EVERY_ROWS = 10

# Arduino struct BleFeatureData와 동일한 순서/크기
# #pragma pack(1)
# uint16_t seq;
# float mean_hr;
# float final_ppg_stress_z;
# float final_gsr_stress_z;
# float integrated_stress_z;
# uint8_t integrated_level;
# uint8_t mode_code;
# uint8_t flags;
BLE_STRUCT_FORMAT = "<HffffBBB"  # little-endian, 21 bytes
BLE_STRUCT_SIZE = struct.calcsize(BLE_STRUCT_FORMAT)

CSV_HEADER = [
    "pc_timestamp",
    "pc_elapsed_sec",
    "seq",
    "mode_code",
    "mode",
    "mean_hr",
    "final_ppg_stress_z",
    "final_gsr_stress_z",
    "integrated_stress_z",
    "integrated_level",
    "integrated_state",
    "flags",
    "ppg_ready",
    "gsr_ready",
    "decision_section",
    "measurement_active",
]

MODE_CODE_MAP = {
    0: "STABILIZING",
    1: "CALIBRATION",
    2: "REST",
    3: "TRANSITION",
    4: "STRESS",
    5: "NO_FINGER_OR_DONE",
}

LEVEL_MAP = {
    0: "",
    1: "Rest",
    2: "Stress",
}


def make_output_paths(save_dir: str, prefix: str):
    Path(save_dir).mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = Path(save_dir) / f"{prefix}_{stamp}"
    return base.with_suffix(".csv"), base.with_suffix(".log"), base.with_suffix(".json")


def wait_for_enter():
    print("====================================")
    print("PPG/GSR Integrated BLE CSV Logger")
    print("====================================")
    print("Arduino 코드가 업로드되어 있고 BLE 광고 이름이 BioGrip_Feature인지 확인하세요.")
    print("측정을 시작하려면 Enter 키를 누르세요.")
    print("Windows에서는 측정 중 x 키, 그 외 환경은 Ctrl+C로 종료합니다.")
    print("====================================")
    input("Enter를 누르면 BLE 연결 후 측정을 시작합니다...")


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


def decode_float_for_csv(value: float):
    # 펌웨어에서 NAN 대신 -1.0을 전송하도록 되어 있어 빈칸으로 저장
    if value == -1.0:
        return ""
    return value


def decode_ble_payload(data: bytes):
    if len(data) < BLE_STRUCT_SIZE:
        raise ValueError(f"BLE payload too short: expected {BLE_STRUCT_SIZE}, got {len(data)}")

    seq, mean_hr, ppg_z, gsr_z, integrated_z, level, mode_code, flags = struct.unpack(
        BLE_STRUCT_FORMAT, data[:BLE_STRUCT_SIZE]
    )

    return {
        "seq": seq,
        "mean_hr": decode_float_for_csv(mean_hr),
        "final_ppg_stress_z": decode_float_for_csv(ppg_z),
        "final_gsr_stress_z": decode_float_for_csv(gsr_z),
        "integrated_stress_z": decode_float_for_csv(integrated_z),
        "integrated_level": level,
        "integrated_state": LEVEL_MAP.get(level, f"UNKNOWN_{level}"),
        "mode_code": mode_code,
        "mode": MODE_CODE_MAP.get(mode_code, f"UNKNOWN_{mode_code}"),
        "flags": flags,
        "ppg_ready": 1 if (flags & 0x01) else 0,
        "gsr_ready": 1 if (flags & 0x02) else 0,
        "decision_section": 1 if (flags & 0x04) else 0,
        "measurement_active": 1 if (flags & 0x08) else 0,
    }


def format_status(row_count: int, row: dict):
    return (
        f"[측정중] rows={row_count} | "
        f"seq={row.get('seq', '-')} | "
        f"mode={row.get('mode', '-')} | "
        f"mean_hr={row.get('mean_hr', '-')} | "
        f"ppg_z={row.get('final_ppg_stress_z', '-')} | "
        f"gsr_z={row.get('final_gsr_stress_z', '-')} | "
        f"integrated_z={row.get('integrated_stress_z', '-')} | "
        f"state={row.get('integrated_state', '-')} | "
        f"flags=0x{int(row.get('flags', 0)):02X} | "
        f"pc_elapsed={float(row.get('pc_elapsed_sec', 0)):.1f}s"
    )


def save_metadata(meta_path, args, csv_path, log_path):
    metadata = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "device_name": args.name,
        "address": args.address,
        "service_uuid": SERVICE_UUID,
        "feature_char_uuid": FEATURE_CHAR_UUID,
        "control_char_uuid": CONTROL_CHAR_UUID,
        "csv_path": str(csv_path),
        "log_path": str(log_path),
        "ble_struct_format": BLE_STRUCT_FORMAT,
        "ble_struct_size": BLE_STRUCT_SIZE,
        "csv_header": CSV_HEADER,
        "notes": [
            "controlPointChar에 1을 write하면 측정 시작, 2를 write하면 측정 정지합니다.",
            "현재 펌웨어의 BLE notify는 mean_hr, 최종 PPG/GSR z, 통합 z, 상태, mode, flags만 전송합니다.",
            "raw_gsr, ir_raw, filtered_ppg, SDNN, RMSSD, SCL, SCR 등 Serial CSV 전체 컬럼은 현재 BLE payload에 포함되어 있지 않으므로 저장할 수 없습니다.",
        ],
    }
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(metadata, f, ensure_ascii=False, indent=2)


async def find_device(name: str, address: str | None, timeout: float):
    if address:
        print(f"BLE 장치 주소로 연결 시도: {address}")
        return address

    print(f"BLE 장치 검색 중: name={name}, timeout={timeout}s")
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: (d.name == name) or (ad.local_name == name),
        timeout=timeout,
    )
    if device is None:
        raise RuntimeError(
            f"BLE 장치 '{name}'를 찾지 못했습니다. 전원이 켜져 있는지, 이미 다른 앱과 연결되어 있지 않은지 확인하세요."
        )
    print(f"BLE 장치 발견: name={device.name}, address={device.address}")
    return device


async def run_logger(args):
    csv_path, log_path, meta_path = make_output_paths(args.save_dir, args.prefix)
    save_metadata(meta_path, args, csv_path, log_path)

    print(f"CSV 저장 예정 파일: {csv_path}")
    print(f"원본 BLE 로그 저장 예정 파일: {log_path}")
    print(f"메타데이터 저장 파일: {meta_path}")
    wait_for_enter()

    target = await find_device(args.name, args.address, args.scan_timeout)

    row_count = 0
    error_count = 0
    start_real_time = None
    stop_event = asyncio.Event()

    def request_stop(*_):
        stop_event.set()

    if os.name != "nt":
        try:
            loop = asyncio.get_running_loop()
            loop.add_signal_handler(signal.SIGINT, request_stop)
            loop.add_signal_handler(signal.SIGTERM, request_stop)
        except NotImplementedError:
            pass

    with open(csv_path, "w", newline="", encoding="utf-8-sig") as csv_file, \
         open(log_path, "w", encoding="utf-8") as log_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_HEADER)
        writer.writeheader()
        csv_file.flush()

        async with BleakClient(target) as client:
            print("\nBLE 연결 완료")
            if not client.is_connected:
                raise RuntimeError("BLE 연결에 실패했습니다.")

            def notification_handler(sender, data: bytearray):
                nonlocal row_count, error_count, start_real_time
                recv_time = datetime.now().isoformat(timespec="milliseconds")
                log_file.write(f"{recv_time}\t{sender}\t{bytes(data).hex()}\n")

                try:
                    decoded = decode_ble_payload(bytes(data))
                    if start_real_time is None:
                        start_real_time = time.time()
                    elapsed = time.time() - start_real_time
                    row = {
                        "pc_timestamp": recv_time,
                        "pc_elapsed_sec": f"{elapsed:.3f}",
                        **decoded,
                    }
                    writer.writerow(row)
                    row_count += 1

                    if row_count % args.status_every_rows == 0:
                        csv_file.flush()
                        log_file.flush()
                        print(format_status(row_count, row))

                except Exception as e:
                    error_count += 1
                    log_file.write(f"{recv_time}\tDECODE_ERROR\t{repr(e)}\n")
                    print(f"[SKIP: BLE 데이터 해석 실패] {e}")

            await client.start_notify(FEATURE_CHAR_UUID, notification_handler)
            await client.write_gatt_char(CONTROL_CHAR_UUID, bytes([1]), response=True)
            print("MCU에 측정 시작 명령 전송 완료: 1")
            print("BLE notify 수신 대기 중...")
            print("====================================")

            try:
                while not stop_event.is_set():
                    if windows_stop_key_pressed():
                        print(f"\n[{STOP_KEY}] 입력 확인 → 측정 종료")
                        break
                    await asyncio.sleep(0.1)
            except KeyboardInterrupt:
                print("\n사용자 중단")
            finally:
                try:
                    await client.write_gatt_char(CONTROL_CHAR_UUID, bytes([2]), response=True)
                    print("MCU에 측정 정지 명령 전송 완료: 2")
                except Exception:
                    pass
                try:
                    await client.stop_notify(FEATURE_CHAR_UUID)
                except Exception:
                    pass

    print("\n====================================")
    print("BLE CSV 저장 종료")
    print(f"저장 파일: {csv_path}")
    print(f"원본 로그: {log_path}")
    print(f"저장 row 수: {row_count}")
    print(f"BLE 해석 실패 수: {error_count}")
    print("====================================")


def main():
    parser = argparse.ArgumentParser(description="PPG/GSR Integrated BLE CSV Logger")
    parser.add_argument("--name", default=DEVICE_NAME, help="BLE device name")
    parser.add_argument("--address", default=None, help="BLE device address. 알고 있으면 검색 대신 직접 연결")
    parser.add_argument("--scan-timeout", type=float, default=SCAN_TIMEOUT_SEC, help="BLE scan timeout seconds")
    parser.add_argument("--save-dir", default=SAVE_DIR, help="CSV 저장 폴더")
    parser.add_argument("--prefix", default=FILE_PREFIX, help="저장 파일 prefix")
    parser.add_argument("--status-every-rows", type=int, default=STATUS_EVERY_ROWS, help="상태 출력 row 간격")
    args = parser.parse_args()

    try:
        asyncio.run(run_logger(args))
    except RuntimeError as e:
        print("\n오류 발생")
        print(e)
        sys.exit(1)
    except Exception as e:
        print("\n예상하지 못한 오류 발생")
        print(e)
        sys.exit(1)


if __name__ == "__main__":
    main()
