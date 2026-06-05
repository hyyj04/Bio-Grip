import sys
import time
import asyncio
import platform
import subprocess
import csv
import os
import numpy as np
from datetime import datetime
import pyqtgraph as pg
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QSystemTrayIcon, QMenu, QAction
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QIcon, QPixmap, QColor
from ble_manager import BLEManager

def send_notification(title: str, message: str) -> None:
    os_name = platform.system()
    if os_name == "Darwin":
        try:
            script = f'display notification "{message}" with title "{title}"'
            subprocess.run(["osascript", "-e", script], check=False)
        except Exception as e:
            print(f"[알림 오류 macOS] {e}")
    elif os_name == "Windows":
        try:
            from winotify import Notification, audio
            toast = Notification(app_id="BioClick", title=title,
                                 msg=message, duration="short")
            toast.set_audio(audio.Default, loop=False)
            toast.show()
        except ImportError:
            print(f"[알림] {title}: {message}")
            print("  → pip install winotify 로 설치하세요")
        except Exception as e:
            print(f"[알림 오류 Windows] {e}")
    else:
        print(f"[알림] {title}: {message}")


class BLEThread(QThread):
    connected      = pyqtSignal(bool)
    data_received  = pyqtSignal(float, float, float, float, float, float)
    error_occurred = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.ble   = BLEManager(
            data_callback=self._on_data,
            connected_callback=self._on_connected
        )
        self._loop = None

    def _on_data(self, hr, ppg, gsr, final, u1, u2):
        self.data_received.emit(hr, ppg, gsr, final, u1, u2)

    def _on_connected(self, state: bool):
        self.connected.emit(state)

    def run(self):
        if platform.system() == "Windows":
            self._loop = asyncio.ProactorEventLoop()
            asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())
        else:
            self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            result = self._loop.run_until_complete(self.ble.scan_and_connect())
            if not result:
                self.connected.emit(False)
                self.error_occurred.emit("장치를 찾지 못했습니다.")
            self._loop.run_forever()
        except Exception as e:
            print(f"[BLE 스레드 오류] {e}")
            self.error_occurred.emit(str(e))
        finally:
            self._loop.close()

    def stop(self):
        if self._loop and self._loop.is_running():
            future = asyncio.run_coroutine_threadsafe(
                self.ble.disconnect(), self._loop)
            try:
                future.result(timeout=3)
            except Exception:
                pass
            finally:
                self._loop.call_soon_threadsafe(self._loop.stop)


def create_tray_icon_image(color: str = "#a6e3a1") -> "Image.Image":
    from PIL import Image, ImageDraw
    size = 64
    img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    draw.ellipse([2, 2, size - 2, size - 2], fill=color, outline="white", width=3)
    return img

def make_tray_icon() -> QIcon:
    from PIL import Image
    import io
    img = create_tray_icon_image("#a6e3a1")
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    buf.seek(0)
    px = QPixmap()
    px.loadFromData(buf.read(), "PNG")
    return QIcon(px)


class MainWindow(QMainWindow):

    NOTIFICATION_COOLDOWN  = 60
    HISTORY_SIZE           = 100
    LEVEL_MAP = {
        0: "--",
        1: "Rest",
        2: "Stress",
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle("BioClick - 스트레스 모니터링")
        self.setGeometry(100, 100, 1000, 700)
        self.setStyleSheet("background-color: #1e1e2e;")

        self.stress_history         = np.zeros(self.HISTORY_SIZE)
        self.ble_connected          = False
        self.ble_thread             = None
        self.last_notification_time = 0.0

        self.csv_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            datetime.now().strftime("bioclick_%Y%m%d_%H%M%S.csv")
        )
        self._init_csv()

        self._build_ui()
        self._build_tray()

    def _init_csv(self):
        """CSV 파일 생성 및 헤더 작성"""
        with open(self.csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp", "mean_hr", "ppg_z",
                "gsr_z", "integrated_z", "state"
            ])
        print(f"[CSV] 저장 파일: {self.csv_path}")

    def _save_csv(self, hr, ppg_score, gsr_score, final_score, state):
        """데이터 한 행 저장"""
        try:
            with open(self.csv_path, "a", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow([
                    datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    round(hr, 1),
                    round(ppg_score, 2) if ppg_score == ppg_score else "--",
                    round(gsr_score, 2) if gsr_score == gsr_score else "--",
                    round(final_score, 2) if final_score == final_score else "--",
                    state
                ])
        except Exception as e:
            print(f"[CSV] 저장 오류: {e}")

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setSpacing(10)
        layout.setContentsMargins(16, 16, 16, 16)

        title = QLabel("🖱 BioClick  스트레스 모니터링")
        title.setFont(QFont("Arial", 20, QFont.Bold))
        title.setStyleSheet("color: #cdd6f4;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        self.stress_plot = pg.PlotWidget(title="스트레스 z-score 변화")
        self.stress_plot.setBackground("#181825")
        self.stress_plot.setYRange(-3, 3)
        self.stress_plot.setMaximumHeight(200)
        self.stress_curve = self.stress_plot.plot(
            pen=pg.mkPen("#f38ba8", width=2))
        layout.addWidget(self.stress_plot)

        style = ("color: #cdd6f4; font-size: 17px; padding: 10px;"
                 "background-color: #181825; border-radius: 6px;")

        row1 = QHBoxLayout()
        self.hr_label         = QLabel("Mean HR\n--")
        self.scr_count_label  = QLabel("State\n--")
        self.ble_status_label = QLabel("BLE\n대기 중")
        for lbl in [self.hr_label, self.scr_count_label, self.ble_status_label]:
            lbl.setStyleSheet(style)
            lbl.setAlignment(Qt.AlignCenter)
            row1.addWidget(lbl)

        layout.addLayout(row1)

        self.stress_label = QLabel("스트레스 지수: --")
        self.stress_label.setFont(QFont("Arial", 28, QFont.Bold))
        self.stress_label.setAlignment(Qt.AlignCenter)
        self.stress_label.setStyleSheet("color: #a6e3a1; padding: 10px;")
        layout.addWidget(self.stress_label)

        self.status_label = QLabel("상태: 대기 중")
        self.status_label.setStyleSheet("color: #f9e2af; font-size: 14px;")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        self.btn = QPushButton("BLE 연결")
        self.btn.setFixedHeight(36)
        self.btn.setStyleSheet(
            "QPushButton { background-color: #89b4fa; color: #1e1e2e;"
            "border-radius: 6px; font-weight: bold; }"
            "QPushButton:disabled { background-color: #585b70; color: #6c7086; }")
        self.btn.clicked.connect(self._toggle_ble)
        layout.addWidget(self.btn)

    def _build_tray(self):
        self.tray = QSystemTrayIcon(make_tray_icon(), self)
        self.tray.setToolTip("BioClick 스트레스 모니터링")
        menu = QMenu()
        show_action = QAction("열기", self)
        quit_action = QAction("종료", self)
        show_action.triggered.connect(self.show)
        quit_action.triggered.connect(QApplication.quit)
        menu.addAction(show_action)
        menu.addSeparator()
        menu.addAction(quit_action)
        self.tray.setContextMenu(menu)
        self.tray.activated.connect(self._on_tray_activated)
        self.tray.show()

    def _on_tray_activated(self, reason):
        if reason == QSystemTrayIcon.DoubleClick:
            self.show()
            self.activateWindow()

    def _update_ui(self, hr, ppg_score, gsr_score, final_score, level_code, _u2):
        z = final_score if final_score == final_score else 0.0  # NaN이면 0
        self.stress_history     = np.roll(self.stress_history, -1)
        self.stress_history[-1] = z
        self.stress_curve.setData(self.stress_history)

        state = self.LEVEL_MAP.get(int(level_code), "--")

        self.hr_label.setText(f"Mean HR\n{hr:.1f} BPM")
        self.scr_count_label.setText(f"State\n{state}")
        self.ble_status_label.setText("BLE\n수신 중")
        self.stress_label.setText(f"상태: {state}")
        self._update_stress_color(level_code)

        if self.ble_connected:
            self._save_csv(hr, ppg_score, gsr_score, final_score, state)

        if int(level_code) == 2:
            self._send_stress_notification()

    def _update_stress_color(self, level_code):
        if int(level_code) == 2:
            color = "#f38ba8"
        elif int(level_code) == 1:
            color = "#a6e3a1"
        else:
            color = "#cdd6f4"
        self.stress_label.setStyleSheet(
            f"color: {color}; font-size: 28px; font-weight: bold; padding: 10px;")

    def _send_stress_notification(self):
        now = time.time()
        if now - self.last_notification_time >= self.NOTIFICATION_COOLDOWN:
            self.last_notification_time = now
            msg = "스트레스 상태가 감지되었습니다. 잠시 휴식을 취하세요."
            if platform.system() == "Darwin":
                send_notification("BioClick 스트레스 경고", msg)
            else:
                self.tray.showMessage("BioClick 스트레스 경고", msg,
                                      QSystemTrayIcon.Warning, 3000)

    def _toggle_ble(self):
        if self.ble_connected:
            self._disconnect_ble()
        else:
            self._connect_ble()

    def _connect_ble(self):
        self.btn.setText("연결 중...")
        self.btn.setEnabled(False)
        self.status_label.setText("상태: 스캔 중...")
        self.status_label.setStyleSheet("color: #f9e2af; font-size: 14px;")
        self.ble_thread = BLEThread()
        self.ble_thread.connected.connect(self._on_ble_connected)
        self.ble_thread.data_received.connect(self._on_ble_data)
        self.ble_thread.error_occurred.connect(self._on_ble_error)
        self.ble_thread.start()

    def _disconnect_ble(self):
        if self.ble_thread:
            self.ble_thread.stop()
            self.ble_thread.wait(3000)
        self.ble_connected = False
        self.ble_thread    = None
        self.btn.setText("BLE 연결")
        self.btn.setEnabled(True)
        self.btn.setStyleSheet(
            "QPushButton { background-color: #89b4fa; color: #1e1e2e;"
            "border-radius: 6px; font-weight: bold; }")
        self.ble_status_label.setText("BLE\n대기 중")
        self.status_label.setText("상태: 연결 해제됨")
        self.status_label.setStyleSheet("color: #f9e2af; font-size: 14px;")

    def _on_ble_connected(self, success: bool):
        if success:
            self.ble_connected = True
            self.btn.setText("BLE 연결 해제")
            self.btn.setEnabled(True)
            self.btn.setStyleSheet(
                "QPushButton { background-color: #a6e3a1; color: #1e1e2e;"
                "border-radius: 6px; font-weight: bold; }")
            self.ble_status_label.setText("BLE\n수신 중")
            self.status_label.setText("상태: 연결됨")
            self.status_label.setStyleSheet("color: #a6e3a1; font-size: 14px;")
        else:
            self.ble_connected = False
            self.ble_status_label.setText("BLE\n연결 끊김")
            self.btn.setText("BLE 연결")
            self.btn.setEnabled(True)
            self.btn.setStyleSheet(
                "QPushButton { background-color: #89b4fa; color: #1e1e2e;"
                "border-radius: 6px; font-weight: bold; }")
            self.status_label.setText("상태: 연결 끊김 — 버튼을 눌러 다시 연결하세요")
            self.status_label.setStyleSheet("color: #f38ba8; font-size: 14px;")

    def _on_ble_data(self, hr, ppg, gsr, final, u1, u2):
        self._update_ui(hr, ppg, gsr, final, u1, u2)

    def _on_ble_error(self, msg):
        self.ble_connected = False
        self.status_label.setText(f"상태: BLE 오류 - {msg[:40]}")
        self.status_label.setStyleSheet("color: #f38ba8; font-size: 14px;")
        self.btn.setText("BLE 연결")
        self.btn.setEnabled(True)

    def closeEvent(self, event):
        event.ignore()
        self.hide()
        self.tray.showMessage("BioClick", "트레이에서 계속 실행 중입니다.",
                              QSystemTrayIcon.Information, 2000)

if __name__ == "__main__":
    if platform.system() == "Windows":
        asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setQuitOnLastWindowClosed(False)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
