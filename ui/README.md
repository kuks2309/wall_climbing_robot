# Robotmaster C620 제어 UI

Qt5로 제작된 서보 모터 제어 UI입니다.

## 파일 설명

- `servo_control.ui`: Qt Designer로 제작된 UI 파일

## Python에서 사용하는 방법

### 1. 필요한 패키지 설치

```bash
pip install PyQt5 pyserial
```

### 2. UI 파일 로드 예제

```python
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer
import serial
import sys
import struct

class ServoControlApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        # UI 파일 로드
        uic.loadUi('servo_control.ui', self)

        self.serial_port = None

        # 시그널 연결
        self.setup_connections()

    def setup_connections(self):
        # 슬라이더 연결
        self.slider_servo1.valueChanged.connect(self.update_servo1)
        self.slider_servo2.valueChanged.connect(self.update_servo2)

        # 스핀박스 연결
        self.spinBox_servo1.valueChanged.connect(self.slider_servo1.setValue)
        self.spinBox_servo2.valueChanged.connect(self.slider_servo2.setValue)

        # 버튼 연결
        self.btn_connect.clicked.connect(self.connect_serial)
        self.btn_disconnect.clicked.connect(self.disconnect_serial)

        # 서보1 프리셋 버튼
        self.btn_servo1_min.clicked.connect(lambda: self.slider_servo1.setValue(1000))
        self.btn_servo1_center.clicked.connect(lambda: self.slider_servo1.setValue(1500))
        self.btn_servo1_max.clicked.connect(lambda: self.slider_servo1.setValue(2000))

        # 서보2 프리셋 버튼
        self.btn_servo2_min.clicked.connect(lambda: self.slider_servo2.setValue(1000))
        self.btn_servo2_center.clicked.connect(lambda: self.slider_servo2.setValue(1500))
        self.btn_servo2_max.clicked.connect(lambda: self.slider_servo2.setValue(2000))

        # 통합 제어 버튼
        self.btn_both_center.clicked.connect(lambda: self.set_both(1500, 1500))
        self.btn_both_min.clicked.connect(lambda: self.set_both(1000, 1000))
        self.btn_both_max.clicked.connect(lambda: self.set_both(2000, 2000))
        self.btn_left_turn.clicked.connect(lambda: self.set_both(2000, 1000))
        self.btn_right_turn.clicked.connect(lambda: self.set_both(1000, 2000))
        self.btn_stop.clicked.connect(lambda: self.set_both(1500, 1500))

        # 로그 지우기
        self.btn_clear_log.clicked.connect(self.textEdit_log.clear)

    def update_servo1(self, value):
        self.label_servo1_value.setText(f"펄스 폭: {value} μs")
        self.spinBox_servo1.setValue(value)
        self.send_command()

    def update_servo2(self, value):
        self.label_servo2_value.setText(f"펄스 폭: {value} μs")
        self.spinBox_servo2.setValue(value)
        self.send_command()

    def set_both(self, servo1_val, servo2_val):
        self.slider_servo1.setValue(servo1_val)
        self.slider_servo2.setValue(servo2_val)

    def connect_serial(self):
        try:
            port = self.comboBox_port.currentText()
            baudrate = int(self.comboBox_baudrate.currentText())

            self.serial_port = serial.Serial(port, baudrate, timeout=1)

            self.label_status.setText("상태: 연결됨")
            self.label_status.setStyleSheet("color: green; font-weight: bold;")

            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(True)

            self.log_message("시리얼 포트 연결 성공")

        except Exception as e:
            self.log_message(f"연결 실패: {str(e)}")

    def disconnect_serial(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

        self.label_status.setText("상태: 연결 안됨")
        self.label_status.setStyleSheet("color: red; font-weight: bold;")

        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)

        self.log_message("시리얼 포트 연결 해제")

    def send_command(self):
        if not self.serial_port or not self.serial_port.is_open:
            return

        servo1_value = self.slider_servo1.value()
        servo2_value = self.slider_servo2.value()

        # 프로토콜: # [servo1_high] [servo1_low] [servo2_high] [servo2_low] *
        servo1_bytes = struct.pack('>H', servo1_value)  # Big-endian short
        servo2_bytes = struct.pack('>H', servo2_value)

        command = b'#' + servo1_bytes + servo2_bytes + b'*'

        try:
            self.serial_port.write(command)

            # 프로토콜 표시 업데이트
            protocol_str = f"# {servo1_bytes[0]:02X} {servo1_bytes[1]:02X} {servo2_bytes[0]:02X} {servo2_bytes[1]:02X} *"
            self.lineEdit_protocol.setText(protocol_str)

            self.log_message(f"전송: Servo1={servo1_value}μs, Servo2={servo2_value}μs")

        except Exception as e:
            self.log_message(f"전송 오류: {str(e)}")

    def log_message(self, message):
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.textEdit_log.append(f"[{timestamp}] {message}")

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = ServoControlApp()
    window.show()
    sys.exit(app.exec_())
```

## UI 구성 요소

### 위젯 이름과 기능

#### 시리얼 연결
- `comboBox_port`: 시리얼 포트 선택
- `comboBox_baudrate`: Baud Rate 선택 (기본: 9600)
- `btn_connect`: 연결 버튼
- `btn_disconnect`: 연결 해제 버튼
- `label_status`: 연결 상태 표시

#### 서보 모터 1 제어
- `slider_servo1`: 서보1 슬라이더 (1000-2000)
- `spinBox_servo1`: 서보1 스핀박스
- `label_servo1_value`: 서보1 현재 값 표시
- `btn_servo1_min`: 최소값 버튼
- `btn_servo1_center`: 중앙값 버튼
- `btn_servo1_max`: 최대값 버튼

#### 서보 모터 2 제어
- `slider_servo2`: 서보2 슬라이더 (1000-2000)
- `spinBox_servo2`: 서보2 스핀박스
- `label_servo2_value`: 서보2 현재 값 표시
- `btn_servo2_min`: 최소값 버튼
- `btn_servo2_center`: 중앙값 버튼
- `btn_servo2_max`: 최대값 버튼

#### 통합 제어
- `btn_both_center`: 양쪽 중앙
- `btn_both_min`: 양쪽 최소
- `btn_both_max`: 양쪽 최대
- `btn_left_turn`: 좌회전
- `btn_right_turn`: 우회전
- `btn_stop`: 정지

#### 프로토콜 정보
- `lineEdit_protocol`: 전송 프로토콜 표시

#### 로그
- `textEdit_log`: 로그 메시지 표시
- `btn_clear_log`: 로그 지우기

## 프로토콜 형식

아두이노 코드에서 사용하는 프로토콜:

```
# [servo1_high] [servo1_low] [servo2_high] [servo2_low] *
```

- 시작 마커: `#` (0x23)
- 서보1 펄스 폭: 2바이트 (Big-endian)
- 서보2 펄스 폭: 2바이트 (Big-endian)
- 종료 마커: `*` (0x2A)

예시: 양쪽 1500μs → `# 05 DC 05 DC *`
