from bleak import BleakClient, BleakScanner
from typing import Callable
import asyncio
import struct

SERVICE_UUID  = "19B10000-E8F2-537E-4F6C-D104768A1214"
CHAR_UUID     = "19B10001-E8F2-537E-4F6C-D104768A1214"
CONTROL_UUID  = "19B10002-E8F2-537E-4F6C-D104768A1214"

DEVICE_KEYWORDS = ["biogrip", "bioclick", "arduino"]

STRUCT_FMT  = '<HffffBBB'
STRUCT_SIZE = struct.calcsize(STRUCT_FMT)

LEVEL_MAP = {
    0: "--",
    1: "Rest",
    2: "Stress",
}

MODE_MAP = {
    0: "STABILIZING",
    1: "CALIBRATION",
    2: "REST",
    3: "TRANSITION",
    4: "STRESS",
    5: "OTHER",
}


class BLEManager:
    def __init__(self, data_callback: Callable,
                 connected_callback: Callable = None) -> None:
        self.data_callback      = data_callback
        self.connected_callback = connected_callback
        self.client             = None
        self.connected          = False
        self._target_address    = None
        self._running           = False

    async def scan_and_connect(self) -> bool:
        self._running = True

        address = await self._scan()
        if address is None:
            return False
        self._target_address = address

        success = await self._connect(self._target_address)
        if not success:
            return False

        if self.connected_callback:
            self.connected_callback(True)

        while self._running and self.connected:
            await asyncio.sleep(1)

        if self.connected_callback:
            self.connected_callback(False)

        return True

    async def _scan(self) -> str | None:
        print("[BLE] 스캔 시작...")
        try:
            devices = await BleakScanner.discover(
                timeout=10.0, return_adv=True)
        except Exception as e:
            print(f"[BLE] 스캔 실패: {e}")
            return None

        for address, (device, adv) in devices.items():
            name = (device.name or "").lower()
            if name and any(kw in name for kw in DEVICE_KEYWORDS):
                print(f"[BLE] 이름으로 발견: {device.name} ({address})")
                return address
            adv_uuids = [str(u).lower() for u in (adv.service_uuids or [])]
            if SERVICE_UUID.lower() in adv_uuids:
                print(f"[BLE] UUID로 발견: {device.name or 'Unknown'} ({address})")
                return address

        print("[BLE] BioClick 장치를 찾지 못했습니다.")
        return None

    async def _connect(self, address: str) -> bool:
        try:
            self.client = BleakClient(
                address,
                disconnected_callback=self._on_disconnected)
            await self.client.connect()
            self.connected = True
            print("[BLE] 연결 성공!")

            await self.client.start_notify(CHAR_UUID, self._on_data_received)

            try:
                await self.client.write_gatt_char(
                    CONTROL_UUID, bytes([0x01]), response=True)
                print("[BLE] 측정 시작 명령 전송 완료 (0x01)")
            except Exception as e:
                print(f"[BLE] 측정 시작 명령 전송 실패: {e}")

            return True
        except Exception as e:
            print(f"[BLE] 연결 실패: {e}")
            self.client    = None
            self.connected = False
            return False

    def _on_disconnected(self, client) -> None:
        print("[BLE] 장치 연결 끊김")
        self.connected = False

    def _on_data_received(self, sender, data: bytearray) -> None:
        try:
            print(f"[BLE] 수신 {len(data)}바이트: {data.hex()}")

            if len(data) < STRUCT_SIZE:
                print(f"[BLE] 데이터 크기 오류: {len(data)} < {STRUCT_SIZE}")
                return

            (seq, mean_hr, ppg_z, gsr_z, integrated_z,
             level_code, mode_code, flags) = struct.unpack_from(STRUCT_FMT, data)

            mode      = MODE_MAP.get(mode_code, "UNKNOWN")
            level_str = LEVEL_MAP.get(level_code, "--")

            if mean_hr      < 0: mean_hr      = 0.0
            if ppg_z        <= -1.0: ppg_z        = float('nan')
            if gsr_z        <= -1.0: gsr_z        = float('nan')
            if integrated_z <= -1.0: integrated_z = float('nan')

            print(f"[BLE] seq={seq} mode={mode} "
                  f"HR={mean_hr:.0f} PPG_z={ppg_z:.2f} "
                  f"GSR_z={gsr_z:.2f} 통합_z={integrated_z:.2f} "
                  f"상태={level_str}")

            self.data_callback(
                mean_hr, ppg_z, gsr_z,
                integrated_z, float(level_code), float(mode_code))

        except Exception as e:
            print(f"[BLE] 파싱 오류: {e} / 원본: {data.hex()}")

    async def disconnect(self) -> None:
        self._running = False
        if self.client and self.connected:
            try:
                await self.client.disconnect()
                print("[BLE] 연결 해제 완료")
            except Exception as e:
                print(f"[BLE] 해제 중 오류: {e}")
            finally:
                self.connected = False
                self.client    = None
