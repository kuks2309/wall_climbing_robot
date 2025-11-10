#!/usr/bin/env python3
"""
Robotmaster C620 서보 모터 제어 애플리케이션

아두이노와 시리얼 통신하여 두 개의 서보 모터를 제어합니다.
프로토콜: # [servo1_high] [servo1_low] [servo2_high] [servo2_low] *
"""

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer
import serial
import serial.tools.list_ports
import sys
import struct
import os
from datetime import datetime


class ServoControlApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        # UI 파일 경로 설정
        ui_path = os.path.join(os.path.dirname(__file__), '..', 'ui', 'servo_control.ui')

        # UI 파일 로드
        uic.loadUi(ui_path, self)

        self.serial_port = None

        # 시그널 연결
        self.setup_connections()

        # 사용 가능한 시리얼 포트 목록 업데이트
        self.refresh_serial_ports()

        # 초기 로그 메시지
        self.log_message("=" * 50)
        self.log_message("Robotmaster C620 제어 프로그램 시작")
        self.log_message("버전: 1.0")
        self.log_message("=" * 50)
        self.log_message("✓ 시스템 준비 완료")

    def setup_connections(self):
        """모든 위젯의 시그널을 연결합니다."""

        # 슬라이더 연결
        self.slider_servo1.valueChanged.connect(self.update_servo1)
        self.slider_servo2.valueChanged.connect(self.update_servo2)

        # 스핀박스 연결 (슬라이더와 동기화)
        self.spinBox_servo1.valueChanged.connect(self.slider_servo1.setValue)
        self.spinBox_servo2.valueChanged.connect(self.slider_servo2.setValue)

        # 연결 버튼
        self.btn_connect.clicked.connect(self.connect_serial)
        self.btn_disconnect.clicked.connect(self.disconnect_serial)

        # 서보1 프리셋 버튼
        self.btn_servo1_min.clicked.connect(lambda: self.slider_servo1.setValue(1000))
        self.btn_servo1_center.clicked.connect(lambda: self.slider_servo1.setValue(1444))
        self.btn_servo1_max.clicked.connect(lambda: self.slider_servo1.setValue(2000))

        # 서보2 프리셋 버튼
        self.btn_servo2_min.clicked.connect(lambda: self.slider_servo2.setValue(1000))
        self.btn_servo2_center.clicked.connect(lambda: self.slider_servo2.setValue(1444))
        self.btn_servo2_max.clicked.connect(lambda: self.slider_servo2.setValue(2000))

        # 통합 제어 버튼
        self.btn_both_center.clicked.connect(lambda: self.set_both(1444, 1444))
        self.btn_both_min.clicked.connect(lambda: self.set_both(1000, 1000))
        self.btn_both_max.clicked.connect(lambda: self.set_both(2000, 2000))
        self.btn_left_turn.clicked.connect(lambda: self.set_both(2000, 1000))
        self.btn_right_turn.clicked.connect(lambda: self.set_both(1000, 2000))
        self.btn_stop.clicked.connect(lambda: self.set_both(1444, 1444))

        # 로그 지우기
        self.btn_clear_log.clicked.connect(self.textEdit_log.clear)

        # 메뉴 액션
        self.action_exit.triggered.connect(self.close)
        self.action_about.triggered.connect(self.show_about)

    def refresh_serial_ports(self):
        """사용 가능한 시리얼 포트 목록을 업데이트합니다."""
        self.comboBox_port.clear()
        ports = serial.tools.list_ports.comports()

        self.log_message("=== 시리얼 포트 검색 ===")
        for port in ports:
            self.comboBox_port.addItem(port.device)
            self.log_message(f"발견: {port.device} - {port.description}")

        if not ports:
            self.comboBox_port.addItem("포트 없음")
            self.log_message("❌ 사용 가능한 시리얼 포트가 없습니다")
        else:
            self.log_message(f"✓ 총 {len(ports)}개 포트 발견")

    def update_servo1(self, value):
        """서보1 값이 변경되었을 때 호출됩니다."""
        self.label_servo1_value.setText(f"펄스 폭: {value} μs")
        self.spinBox_servo1.blockSignals(True)
        self.spinBox_servo1.setValue(value)
        self.spinBox_servo1.blockSignals(False)
        self.send_command()

    def update_servo2(self, value):
        """서보2 값이 변경되었을 때 호출됩니다."""
        self.label_servo2_value.setText(f"펄스 폭: {value} μs")
        self.spinBox_servo2.blockSignals(True)
        self.spinBox_servo2.setValue(value)
        self.spinBox_servo2.blockSignals(False)
        self.send_command()

    def set_both(self, servo1_val, servo2_val):
        """두 서보 모터의 값을 동시에 설정합니다."""
        self.slider_servo1.setValue(servo1_val)
        self.slider_servo2.setValue(servo2_val)
        self.log_message(f"양쪽 설정: Servo1={servo1_val}μs, Servo2={servo2_val}μs")

    def connect_serial(self):
        """시리얼 포트에 연결합니다."""
        try:
            port = self.comboBox_port.currentText()

            if port == "포트 없음":
                self.log_message("❌ 연결할 포트가 없습니다")
                return

            baudrate = int(self.comboBox_baudrate.currentText())

            self.log_message(f"[연결 시도] {port} @ {baudrate} bps")

            self.serial_port = serial.Serial(port, baudrate, timeout=1)

            self.label_status.setText("상태: 연결됨")
            self.label_status.setStyleSheet("color: green; font-weight: bold;")

            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(True)
            self.comboBox_port.setEnabled(False)
            self.comboBox_baudrate.setEnabled(False)

            self.log_message(f"✓ 시리얼 포트 연결 성공!")
            self.log_message(f"  - 포트: {port}")
            self.log_message(f"  - Baud Rate: {baudrate}")
            self.log_message(f"  - Timeout: 1초")

            # 연결 후 초기 명령 전송
            self.log_message("[초기화] 초기 명령 전송 중...")
            self.send_command()

        except Exception as e:
            self.log_message(f"❌ 연결 실패: {str(e)}")
            QtWidgets.QMessageBox.critical(self, "연결 오류", f"시리얼 포트 연결에 실패했습니다.\n{str(e)}")

    def disconnect_serial(self):
        """시리얼 포트 연결을 해제합니다."""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

        self.label_status.setText("상태: 연결 안됨")
        self.label_status.setStyleSheet("color: red; font-weight: bold;")

        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.comboBox_port.setEnabled(True)
        self.comboBox_baudrate.setEnabled(True)

        self.log_message("시리얼 포트 연결 해제")

    def send_command(self):
        """
        현재 서보 값을 아두이노로 전송합니다.

        프로토콜: # [servo1_high] [servo1_low] [servo2_high] [servo2_low] *
        - 시작 마커: # (0x23)
        - 서보1 펄스 폭: 2바이트 (Big-endian)
        - 서보2 펄스 폭: 2바이트 (Big-endian)
        - 종료 마커: * (0x2A)
        """
        if not self.serial_port or not self.serial_port.is_open:
            self.log_message("⚠ 시리얼 포트가 열려있지 않습니다")
            return

        servo1_value = self.slider_servo1.value()
        servo2_value = self.slider_servo2.value()

        # Big-endian 16비트 정수로 변환
        servo1_bytes = struct.pack('>H', servo1_value)
        servo2_bytes = struct.pack('>H', servo2_value)

        # 프로토콜 생성: # + servo1(2bytes) + servo2(2bytes) + *
        command = b'#' + servo1_bytes + servo2_bytes + b'*'

        try:
            # 전송 전 로그
            self.log_message("=" * 50)
            self.log_message("[TX] 데이터 전송 시작")
            self.log_message(f"  Servo1: {servo1_value} μs (0x{servo1_value:04X})")
            self.log_message(f"  Servo2: {servo2_value} μs (0x{servo2_value:04X})")

            # 바이트 분해 로그
            self.log_message(f"  Servo1 bytes: 0x{servo1_bytes[0]:02X} 0x{servo1_bytes[1]:02X}")
            self.log_message(f"  Servo2 bytes: 0x{servo2_bytes[0]:02X} 0x{servo2_bytes[1]:02X}")

            # 전체 패킷 로그
            packet_hex = ' '.join([f'{b:02X}' for b in command])
            self.log_message(f"  전체 패킷: {packet_hex}")
            self.log_message(f"  패킷 길이: {len(command)} 바이트")

            # 시리얼 전송
            bytes_written = self.serial_port.write(command)
            self.serial_port.flush()  # 전송 버퍼 비우기

            self.log_message(f"✓ 전송 완료: {bytes_written} 바이트")
            self.log_message("=" * 50)

            # 프로토콜 표시 업데이트 (16진수)
            protocol_str = f"# {servo1_bytes[0]:02X} {servo1_bytes[1]:02X} {servo2_bytes[0]:02X} {servo2_bytes[1]:02X} *"
            self.lineEdit_protocol.setText(protocol_str)

        except Exception as e:
            self.log_message(f"❌ 전송 오류: {str(e)}")
            self.log_message("=" * 50)
            self.disconnect_serial()

    def log_message(self, message):
        """로그 메시지를 텍스트 편집기에 추가합니다."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.textEdit_log.append(f"[{timestamp}] {message}")

        # 자동 스크롤
        scrollbar = self.textEdit_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def show_about(self):
        """정보 대화상자를 표시합니다."""
        about_text = """
        <h2>Robotmaster C620 제어 패널</h2>
        <p>버전: 1.0</p>
        <p>아두이노와 시리얼 통신하여 DJI RoboMaster C620 ESC를 제어합니다.</p>
        <br>
        <p><b>프로토콜 형식:</b></p>
        <p><code># [servo1_high] [servo1_low] [servo2_high] [servo2_low] *</code></p>
        <br>
        <p>펄스 폭 범위: 1000μs ~ 2000μs</p>
        <p>중립값: 1444μs</p>
        <p>PWM 주기: 15ms (66.7Hz)</p>
        """
        QtWidgets.QMessageBox.about(self, "정보", about_text)

    def closeEvent(self, event):
        """애플리케이션 종료 시 시리얼 포트를 정리합니다."""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        event.accept()


def main():
    """메인 함수"""
    app = QtWidgets.QApplication(sys.argv)

    # 애플리케이션 스타일 설정 (선택사항)
    app.setStyle('Fusion')

    window = ServoControlApp()
    window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
