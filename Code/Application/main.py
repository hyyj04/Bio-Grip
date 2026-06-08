import sys
import time
import asyncio
import platform
import subprocess
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QSystemTrayIcon, QMenu, QAction, QDialog,
    QTableWidget, QTableWidgetItem
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QIcon, QPixmap

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
            toast = Notification(
                app_id="BioClick",
                title=title,
                msg=message,
                duration="short"
            )
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
    connected = pyqtSignal(bool)
    data_received = pyqtSignal(float, float, float, float, float, float)
    error_occurred = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.ble = BLEManager(
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
                self.ble.disconnect(),
                self._loop
            )

            try:
                future.result(timeout=3)
            except Exception:
                pass
            finally:
                self._loop.call_soon_threadsafe(self._loop.stop)


def create_tray_icon_image(color: str = "#a6e3a1"):
    from PIL import Image, ImageDraw

    size = 64
    img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    draw.ellipse(
        [2, 2, size - 2, size - 2],
        fill=color,
        outline="white",
        width=3
    )
    return img


def make_tray_icon() -> QIcon:
    import io

    img = create_tray_icon_image("#a6e3a1")
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    buf.seek(0)

    px = QPixmap()
    px.loadFromData(buf.read(), "PNG")
    return QIcon(px)


class HistoryDialog(QDialog):
    def __init__(self, title, headers, rows):
        super().__init__()

        self.setWindowTitle(title)
        self.setGeometry(200, 200, 820, 420)
        self.setStyleSheet("background-color: #1e1e2e; color: #cdd6f4;")

        layout = QVBoxLayout(self)

        title_label = QLabel(title)
        title_label.setFont(QFont("Arial", 18, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("color: #cdd6f4; padding: 8px;")
        layout.addWidget(title_label)

        table = QTableWidget()
        table.setColumnCount(len(headers))
        table.setRowCount(len(rows))
        table.setHorizontalHeaderLabels(headers)

        table.setStyleSheet(
            "QTableWidget { background-color: #181825; color: #cdd6f4; "
            "gridline-color: #313244; }"
            "QHeaderView::section { background-color: #313244; "
            "color: #cdd6f4; padding: 6px; }"
        )

        for r, row in enumerate(rows):
            for c, value in enumerate(row):
                item = QTableWidgetItem(str(value))
                item.setTextAlignment(Qt.AlignCenter)
                table.setItem(r, c, item)

        table.resizeColumnsToContents()
        layout.addWidget(table)


class MainWindow(QMainWindow):
    NOTIFICATION_COOLDOWN = 60
    MAX_HISTORY = 300

    LEVEL_MAP = {
        0: "--",
        1: "REST",
        2: "STRESS",
    }

    def __init__(self):
        super().__init__()

        self.setWindowTitle("BioClick - 스트레스 모니터링")
        self.setGeometry(100, 100, 900, 540)
        self.setStyleSheet("background-color: #1e1e2e;")

        self.ble_connected = False
        self.ble_thread = None
        self.last_notification_time = 0.0

        self.last_receive_time = None
        self.receive_interval = 0.0

        self.detail_history = []
        self.hr_history = []

        self._build_ui()
        self._build_tray()

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)

        layout = QVBoxLayout(central)
        layout.setSpacing(18)
        layout.setContentsMargins(24, 24, 24, 24)

        title = QLabel("🖱 BioClick 스트레스 모니터링")
        title.setFont(QFont("Arial", 22, QFont.Bold))
        title.setStyleSheet("color: #cdd6f4;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        guide = QLabel("상태 카드를 클릭하면 시간대별 상세 측정 기록을 확인할 수 있습니다.")
        guide.setFont(QFont("Arial", 12))
        guide.setStyleSheet("color: #bac2de;")
        guide.setAlignment(Qt.AlignCenter)
        layout.addWidget(guide)

        self.state_card = QPushButton("현재 상태\n--")
        self.state_card.clicked.connect(self._show_detail_history)
        self._set_card_style(self.state_card, "#313244", "#cdd6f4")
        layout.addWidget(self.state_card)

        info_row = QHBoxLayout()

        self.hr_card = QPushButton("Mean HR\n-- BPM")
        self.hr_card.clicked.connect(self._show_hr_history)
        self._set_card_style(self.hr_card, "#181825", "#89b4fa")

        self.ble_status_label = QLabel("BLE\n대기 중")
        self.ble_status_label.setAlignment(Qt.AlignCenter)
        self.ble_status_label.setStyleSheet(
            "background-color: #181825; color: #cdd6f4; "
            "font-size: 18px; font-weight: bold; "
            "border-radius: 12px; padding: 24px;"
        )

        info_row.addWidget(self.hr_card)
        info_row.addWidget(self.ble_status_label)
        layout.addLayout(info_row)

        self.status_label = QLabel("상태: BLE 연결 전")
        self.status_label.setStyleSheet("color: #f9e2af; font-size: 14px;")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        self.btn = QPushButton("BLE 연결")
        self.btn.setFixedHeight(42)
        self.btn.setStyleSheet(
            "QPushButton { background-color: #89b4fa; color: #1e1e2e; "
            "border-radius: 8px; font-weight: bold; font-size: 15px; }"
            "QPushButton:disabled { background-color: #585b70; color: #6c7086; }"
        )
        self.btn.clicked.connect(self._toggle_ble)
        layout.addWidget(self.btn)

    def _set_card_style(self, widget, bg_color, text_color):
        widget.setMinimumHeight(150)
        widget.setStyleSheet(
            f"QPushButton {{ "
            f"background-color: {bg_color}; "
            f"color: {text_color}; "
            f"border-radius: 16px; "
            f"font-size: 28px; "
            f"font-weight: bold; "
            f"padding: 20px; "
            f"}}"
            f"QPushButton:hover {{ "
            f"background-color: #45475a; "
            f"}}"
        )

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

    def _reset_measurement_records(self):
        self.detail_history.clear()
        self.hr_history.clear()

        self.last_receive_time = None
        self.receive_interval = 0.0

        self.state_card.setText("현재 상태\n--")
        self.hr_card.setText("Mean HR\n-- BPM")
        self.ble_status_label.setText("BLE\n수신 중")

        self._set_card_style(self.state_card, "#313244", "#cdd6f4")

    def _update_ui(self, hr, ppg_score, gsr_score, final_score, level_code, _u2):
        state = self.LEVEL_MAP.get(int(level_code), "--")
        now_text = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        self.hr_card.setText(f"Mean HR\n{hr:.1f} BPM")
        self.ble_status_label.setText(
            f"BLE\n수신 중\n{self.receive_interval:.2f}초"
        )

        self._append_history(now_text, hr, ppg_score, gsr_score, final_score, state)

        self.state_card.setText(f"현재 상태\n{state}")

        if state == "REST":
            self._set_card_style(self.state_card, "#313244", "#a6e3a1")

        elif state == "STRESS":
            self._set_card_style(self.state_card, "#313244", "#f38ba8")
            self._send_stress_notification()

        else:
            self._set_card_style(self.state_card, "#313244", "#cdd6f4")

    def _append_history(self, timestamp, hr, ppg_score, gsr_score, final_score, state):
        if ppg_score == ppg_score:
            ppg_text = f"{ppg_score:.2f}"
        else:
            ppg_text = "--"

        if gsr_score == gsr_score:
            gsr_text = f"{gsr_score:.2f}"
        else:
            gsr_text = "--"

        if final_score == final_score:
            final_text = f"{final_score:.2f}"
        else:
            final_text = "--"

        self.detail_history.append([
            timestamp,
            f"HR={hr:.0f}",
            f"PPG_z={ppg_text}",
            f"GSR_z={gsr_text}",
            f"통합_z={final_text}",
            f"상태={state}"
        ])

        self.hr_history.append([timestamp, f"{hr:.1f} BPM"])

        if len(self.detail_history) > self.MAX_HISTORY:
            self.detail_history.pop(0)

        if len(self.hr_history) > self.MAX_HISTORY:
            self.hr_history.pop(0)

    def _show_detail_history(self):
        dialog = HistoryDialog(
            "상세 측정 기록",
            ["시간", "HR", "PPG_z", "GSR_z", "통합_z", "상태"],
            self.detail_history
        )
        dialog.exec_()

    def _show_hr_history(self):
        dialog = HistoryDialog(
            "Mean HR 시간대별 기록",
            ["시간", "Mean HR"],
            self.hr_history
        )
        dialog.exec_()

    def _send_stress_notification(self):
        now = time.time()

        if now - self.last_notification_time >= self.NOTIFICATION_COOLDOWN:
            self.last_notification_time = now
            msg = "스트레스 상태가 감지되었습니다. 잠시 휴식을 취하세요."

            if platform.system() == "Darwin":
                send_notification("BioClick 스트레스 경고", msg)
            else:
                self.tray.showMessage(
                    "BioClick 스트레스 경고",
                    msg,
                    QSystemTrayIcon.Warning,
                    3000
                )

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
        self.ble_thread = None
        self.last_receive_time = None
        self.receive_interval = 0.0

        self.btn.setText("BLE 연결")
        self.btn.setEnabled(True)
        self.btn.setStyleSheet(
            "QPushButton { background-color: #89b4fa; color: #1e1e2e; "
            "border-radius: 8px; font-weight: bold; font-size: 15px; }"
        )

        self.ble_status_label.setText("BLE\n대기 중")
        self.status_label.setText("상태: 연결 해제됨")
        self.status_label.setStyleSheet("color: #f9e2af; font-size: 14px;")

    def _on_ble_connected(self, success: bool):
        if success:
            self.ble_connected = True

            self._reset_measurement_records()

            self.btn.setText("BLE 연결 해제")
            self.btn.setEnabled(True)
            self.btn.setStyleSheet(
                "QPushButton { background-color: #a6e3a1; color: #1e1e2e; "
                "border-radius: 8px; font-weight: bold; font-size: 15px; }"
            )

            self.status_label.setText("상태: 연결됨")
            self.status_label.setStyleSheet("color: #a6e3a1; font-size: 14px;")

        else:
            self.ble_connected = False

            self.ble_status_label.setText("BLE\n연결 끊김")
            self.btn.setText("BLE 연결")
            self.btn.setEnabled(True)
            self.btn.setStyleSheet(
                "QPushButton { background-color: #89b4fa; color: #1e1e2e; "
                "border-radius: 8px; font-weight: bold; font-size: 15px; }"
            )

            self.status_label.setText("상태: 연결 끊김 — 버튼을 눌러 다시 연결하세요")
            self.status_label.setStyleSheet("color: #f38ba8; font-size: 14px;")

    def _on_ble_data(self, hr, ppg, gsr, final, u1, u2):
        now = time.time()

        if self.last_receive_time is not None:
            self.receive_interval = now - self.last_receive_time
            print(f"[APP] BLE 수신 주기: {self.receive_interval:.2f}초")

        self.last_receive_time = now

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
        self.tray.showMessage(
            "BioClick",
            "트레이에서 계속 실행 중입니다.",
            QSystemTrayIcon.Information,
            2000
        )


if __name__ == "__main__":
    if platform.system() == "Windows":
        asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setQuitOnLastWindowClosed(False)

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())
