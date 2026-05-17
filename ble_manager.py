from bleak import BleakClient, BleakScanner
from typing import Callable
import asyncio

SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"
CHAR_UUID    = "19B10001-E8F2-537E-4F6C-D104768A1214"

DEVICE_KEYWORDS = ["biogrip", "bioclick", "arduino"]


class BLEManager:
    def __init__(self, data_callback: Callable,
                 connected_callback: Callable = None) -> None:
        self.data_callback      = data_callback
        self.connected_callback = connected_callback  # 연결/재연결 시 호출
        self.client             = None
        self.connected          = False
        self._target_address    = None
        self._running           = False

    async def scan_and_connect(self) -> bool:
        self._running = True

        if self._target_address is None:
            address = await self._scan()
            if address is None:
                return False
            self._target_address = address

        while self._running:
            success = await self._connect(self._target_address)
            if not success:
                print("[BLE] 5초 후 재연결 시도...")
                await asyncio.sleep(5)
                continue

            # 연결 성공 → 콜백으로 main.py에 알림
            if self.connected_callback:
                self.connected_callback(True)

            while self._running and self.connected:
                await asyncio.sleep(1)

            if self._running:
                print("[BLE] 연결 끊김 감지 → 재연결 시도...")
                if self.connected_callback:
                    self.connected_callback(False)  # 끊김 상태 알림
                await asyncio.sleep(2)

        return True

    async def _scan(self) -> str | None:
        print("[BLE] 스캔 시작...")
        try:
            devices = await BleakScanner.discover(timeout=10.0)
        except Exception as e:
            print(f"[BLE] 스캔 실패: {e}")
            return None

        for d in devices:
            if d.name and any(kw in d.name.lower() for kw in DEVICE_KEYWORDS):
                print(f"[BLE] 장치 발견: {d.name} ({d.address})")
                return d.address

        print("[BLE] BioClick 장치를 찾지 못했습니다.")
        return None

    async def _connect(self, address: str) -> bool:
        try:
            self.client = BleakClient(
                address,
                disconnected_callback=self._on_disconnected
            )
            await self.client.connect()
            self.connected = True
            print("[BLE] 연결 성공!")
            await self.client.start_notify(CHAR_UUID, self._on_data_received)
            return True
        except Exception as e:
            print(f"[BLE] 연결 실패: {e}")
            self.client    = None
            self.connected = False
            return False

    def _on_disconnected(self, client):
        print("[BLE] 장치 연결 끊김")
        self.connected = False

    def _on_data_received(self, sender, data: bytearray) -> None:
        try:
            text = data.decode("utf-8").strip()
            print(f"[BLE] 수신 원본 : {text}")

            if text.startswith("[STRESS:"):
                return

            parts = text.split(",")

            if len(parts) == 5:
                hr, ppg, gsr, final = (float(p) for p in parts[:4])
                self.data_callback(hr, ppg, gsr, final, 0.0, 0.0)

            elif len(parts) == 6:
                hr, sdnn, rmssd, scl, scr_count, scr_amp = map(float, parts)
                self.data_callback(hr, sdnn, rmssd, scl, scr_count, scr_amp)

            else:
                print(f"[BLE] 알 수 없는 포맷 ({len(parts)}컬럼): {text}")

        except Exception as e:
            print(f"[BLE] 파싱 오류: {e} / 원본: {data}")

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