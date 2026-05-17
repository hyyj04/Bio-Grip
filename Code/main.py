import sys
import time
import asyncio
import subprocess
import numpy as np
import pyqtgraph as pg

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QLabel, QPushButton
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QFont

from signal_processor import SignalProcessor
from ble_manager import BLEManager


pg.setConfigOptions(useOpenGL=False, antialias=False)


def send_mac_notification(title, message):
    try:
        script = f'display notification "{message}" with title "{title}"'
        subprocess.run(["osascript", "-e", script], check=False)
    except Exception as e:
        print(f"알림 전송 오류: {e}")


class BLEThread(QThread):
    connected = pyqtSignal(bool)
    data_received = pyqtSignal(float, float)
    error_occurred = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.loop = None
        self.ble = BLEManager(self.on_data)

    def on_data(self, ppg, gsr):
        self.data_received.emit(ppg, gsr)

    def run(self):
        try:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)

            result = self.loop.run_until_complete(self.ble.scan_and_connect())
            self.connected.emit(result)

            if result:
                self.loop.run_forever()

        except Exception as e:
            self.error_occurred.emit(str(e))

        finally:
            if self.loop is not None:
                try:
                    if self.ble is not None:
                        self.loop.run_until_complete(self.ble.disconnect())
                except Exception as e:
                    print(f"스레드 종료 중 disconnect 오류: {e}")

                try:
                    pending = asyncio.all_tasks(self.loop)
                    for task in pending:
                        task.cancel()

                    if pending:
                        self.loop.run_until_complete(
                            asyncio.gather(*pending, return_exceptions=True)
                        )
                except Exception as e:
                    print(f"pending task 정리 오류: {e}")

                try:
                    self.loop.close()
                except Exception as e:
                    print(f"이벤트 루프 종료 오류: {e}")

    def stop(self):
        try:
            if self.loop is not None:
                future = asyncio.run_coroutine_threadsafe(
                    self.ble.disconnect(),
                    self.loop
                )
                future.result(timeout=5)

                self.loop.call_soon_threadsafe(self.loop.stop)
        except Exception as e:
            print(f"BLEThread stop 오류: {e}")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("BioClick - 스트레스 모니터링")
        self.setGeometry(100, 100, 1000, 750)
        self.setStyleSheet("background-color: #1e1e2e;")

        self.processor = SignalProcessor(fs=50)

        self.ppg_data = np.zeros(200, dtype=float)
        self.gsr_data = np.zeros(200, dtype=float)

        self.ble_connected = False
        self.ble_thread = None

        self.last_notification_time = 0
        self.NOTIFICATION_COOLDOWN = 60

        central = QWidget()
        self.setCentralWidget(central)

        layout = QVBoxLayout(central)
        layout.setSpacing(10)
        layout.setContentsMargins(16, 16, 16, 16)

        title = QLabel("BioClick 스트레스 모니터링")
        title.setFont(QFont("Arial", 18, QFont.Bold))
        title.setStyleSheet("color: #cdd6f4;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        ppg_label = QLabel("PPG 신호 (심박)")
        ppg_label.setStyleSheet("color: #a6e3a1; font-size: 13px;")
        layout.addWidget(ppg_label)

        self.ppg_plot = pg.PlotWidget()
        self.ppg_plot.setBackground("#181825")
        self.ppg_plot.setYRange(-1, 1)
        self.ppg_plot.showGrid(x=True, y=True, alpha=0.2)
        self.ppg_plot.setMaximumHeight(160)
        self.ppg_curve = self.ppg_plot.plot(pen=pg.mkPen("#a6e3a1", width=2))
        layout.addWidget(self.ppg_plot)

        gsr_label = QLabel("GSR 신호 (피부 전도도)")
        gsr_label.setStyleSheet("color: #89b4fa; font-size: 13px;")
        layout.addWidget(gsr_label)

        self.gsr_plot = pg.PlotWidget()
        self.gsr_plot.setBackground("#181825")
        self.gsr_plot.setYRange(-1, 1)
        self.gsr_plot.showGrid(x=True, y=True, alpha=0.2)
        self.gsr_plot.setMaximumHeight(160)
        self.gsr_curve = self.gsr_plot.plot(pen=pg.mkPen("#89b4fa", width=2))
        layout.addWidget(self.gsr_plot)

        ppg_metrics = QHBoxLayout()

        self.hr_label = QLabel("Mean HR: -- BPM")
        self.hr_label.setFont(QFont("Arial", 12))
        self.hr_label.setStyleSheet("color: #a6e3a1;")
        self.hr_label.setAlignment(Qt.AlignCenter)
        ppg_metrics.addWidget(self.hr_label)

        self.sdnn_label = QLabel("SDNN: -- ms")
        self.sdnn_label.setFont(QFont("Arial", 12))
        self.sdnn_label.setStyleSheet("color: #a6e3a1;")
        self.sdnn_label.setAlignment(Qt.AlignCenter)
        ppg_metrics.addWidget(self.sdnn_label)

        self.rmssd_label = QLabel("RMSSD: -- ms")
        self.rmssd_label.setFont(QFont("Arial", 12))
        self.rmssd_label.setStyleSheet("color: #a6e3a1;")
        self.rmssd_label.setAlignment(Qt.AlignCenter)
        ppg_metrics.addWidget(self.rmssd_label)

        layout.addLayout(ppg_metrics)

        gsr_metrics = QHBoxLayout()

        self.scl_label = QLabel("SCL: --")
        self.scl_label.setFont(QFont("Arial", 12))
        self.scl_label.setStyleSheet("color: #89b4fa;")
        self.scl_label.setAlignment(Qt.AlignCenter)
        gsr_metrics.addWidget(self.scl_label)

        self.scr_count_label = QLabel("SCR Count: --")
        self.scr_count_label.setFont(QFont("Arial", 12))
        self.scr_count_label.setStyleSheet("color: #89b4fa;")
        self.scr_count_label.setAlignment(Qt.AlignCenter)
        gsr_metrics.addWidget(self.scr_count_label)

        self.scr_amp_label = QLabel("SCR Amplitude: --")
        self.scr_amp_label.setFont(QFont("Arial", 12))
        self.scr_amp_label.setStyleSheet("color: #89b4fa;")
        self.scr_amp_label.setAlignment(Qt.AlignCenter)
        gsr_metrics.addWidget(self.scr_amp_label)

        layout.addLayout(gsr_metrics)

        bottom = QHBoxLayout()

        self.stress_label = QLabel("스트레스 지수: --")
        self.stress_label.setFont(QFont("Arial", 20, QFont.Bold))
        self.stress_label.setStyleSheet("color: #a6e3a1;")
        self.stress_label.setAlignment(Qt.AlignCenter)
        bottom.addWidget(self.stress_label)

        btn_layout = QVBoxLayout()

        self.btn = QPushButton("BLE 연결")
        self.btn.setStyleSheet("""
            QPushButton {
                background-color: #89b4fa;
                color: #1e1e2e;
                font-size: 14px;
                font-weight: bold;
                padding: 10px 24px;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #74c7ec;
            }
        """)
        self.btn.clicked.connect(self.toggle_ble)
        btn_layout.addWidget(self.btn)

        self.status_label = QLabel("상태: 미연결")
        self.status_label.setStyleSheet("color: #f38ba8; font-size: 11px;")
        self.status_label.setAlignment(Qt.AlignCenter)
        btn_layout.addWidget(self.status_label)

        bottom.addLayout(btn_layout)
        layout.addLayout(bottom)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_dummy_data)
        self.timer.start(50)

    def send_notification(self, stress):
        now = time.time()
        if now - self.last_notification_time < self.NOTIFICATION_COOLDOWN:
            return

        self.last_notification_time = now
        send_mac_notification(
            "BioClick - 휴식 알림",
            f"스트레스 지수가 {stress}입니다. 잠시 휴식을 취하세요!"
        )

    def update_stress_color(self, stress):
        if stress < 40:
            color = "#a6e3a1"
        elif stress < 70:
            color = "#f9e2af"
        else:
            color = "#f38ba8"

        self.stress_label.setStyleSheet(f"color: {color};")

    def toggle_ble(self):
        if self.ble_connected:
            return

        self.btn.setText("연결 중...")
        self.btn.setEnabled(False)
        self.status_label.setText("상태: 스캔 중...")
        self.status_label.setStyleSheet("color: #f9e2af; font-size: 11px;")

        self.ble_thread = BLEThread()
        self.ble_thread.connected.connect(self.on_ble_connected)
        self.ble_thread.data_received.connect(self.on_ble_data)
        self.ble_thread.error_occurred.connect(self.on_ble_error)
        self.ble_thread.start()

    def on_ble_connected(self, success):
        if success:
            self.ble_connected = True
            self.timer.stop()

            self.btn.setText("BLE 연결됨")
            self.btn.setEnabled(False)
            self.btn.setStyleSheet("""
                QPushButton {
                    background-color: #a6e3a1;
                    color: #1e1e2e;
                    font-size: 14px;
                    font-weight: bold;
                    padding: 10px 24px;
                    border-radius: 8px;
                }
            """)

            self.status_label.setText("상태: 연결됨")
            self.status_label.setStyleSheet("color: #a6e3a1; font-size: 11px;")

        else:
            self.btn.setText("BLE 연결")
            self.btn.setEnabled(True)
            self.status_label.setText("상태: 연결 실패 - 다시 시도")
            self.status_label.setStyleSheet("color: #f38ba8; font-size: 11px;")

    def on_ble_error(self, message):
        self.btn.setText("BLE 연결")
        self.btn.setEnabled(True)
        self.status_label.setText(f"상태: 오류 - {message}")
        self.status_label.setStyleSheet("color: #f38ba8; font-size: 11px;")
        print(f"BLE 스레드 오류: {message}")

    def on_ble_data(self, ppg_val, gsr_val):
        self.ppg_data = np.roll(self.ppg_data, -1)
        self.ppg_data[-1] = ppg_val

        self.gsr_data = np.roll(self.gsr_data, -1)
        self.gsr_data[-1] = gsr_val

        self.update_ui()

    def update_dummy_data(self):
        self.ppg_data = np.roll(self.ppg_data, -1)
        self.ppg_data[-1] = np.sin(time.time() * 3)

        self.gsr_data = np.roll(self.gsr_data, -1)
        self.gsr_data[-1] = np.sin(time.time() * 1.5) * 0.5

        self.update_ui()

    def update_ui(self):
        try:
            self.ppg_curve.setData(self.ppg_data)
            self.gsr_curve.setData(self.gsr_data)

            ppg_filtered = self.processor.filter_ppg(self.ppg_data)
            gsr_filtered = self.processor.filter_gsr(self.gsr_data)

            rr_intervals = self.processor.get_rr_intervals(ppg_filtered)

            mean_hr = self.processor.calc_mean_hr(rr_intervals)
            sdnn = self.processor.calc_sdnn(rr_intervals)
            rmssd = self.processor.calc_rmssd(rr_intervals)

            scl = self.processor.calc_scl(gsr_filtered)
            scr_count = self.processor.calc_scr_count(gsr_filtered)
            scr_amp = self.processor.calc_scr_amplitude(gsr_filtered)

            stress = self.processor.calc_stress_index(
                mean_hr, rmssd, scl, scr_count, sdnn, scr_amp
            )

            self.hr_label.setText(f"Mean HR: {mean_hr} BPM")
            self.sdnn_label.setText(f"SDNN: {sdnn} ms")
            self.rmssd_label.setText(f"RMSSD: {rmssd} ms")
            self.scl_label.setText(f"SCL: {scl}")
            self.scr_count_label.setText(f"SCR Count: {scr_count}")
            self.scr_amp_label.setText(f"SCR Amplitude: {scr_amp}")
            self.stress_label.setText(f"스트레스 지수: {stress}")

            self.update_stress_color(stress)

            if stress >= 70:
                self.send_notification(stress)

        except Exception as e:
            print(f"UI 업데이트 오류: {e}")

    def closeEvent(self, event):
        try:
            if self.timer.isActive():
                self.timer.stop()

            if self.ble_thread is not None:
                self.ble_thread.stop()
                self.ble_thread.wait(3000)

        except Exception as e:
            print(f"종: {e}")

        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_()) 