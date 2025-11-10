# Robotmaster C620 제어 스크립트

아두이노와 시리얼 통신하여 서보 모터를 제어하는 Python 애플리케이션입니다.

## 설치 방법

### 1. 필요한 패키지 설치

```bash
pip install -r requirements.txt
```

또는 개별 설치:

```bash
pip install PyQt5 pyserial
```

### 2. 실행 권한 부여 (Linux/Mac)

```bash
chmod +x servo_control_app.py
```

## 실행 방법

```bash
python servo_control_app.py
```

또는 (실행 권한이 있는 경우):

```bash
./servo_control_app.py
```

## 사용 방법

### 1. 시리얼 포트 연결

1. 아두이노를 컴퓨터에 연결합니다
2. 프로그램을 실행하면 사용 가능한 포트 목록이 표시됩니다
3. 올바른 포트를 선택합니다 (예: `/dev/ttyUSB0`, `COM3`)
4. Baud Rate를 선택합니다 (기본값: 9600)
5. "연결" 버튼을 클릭합니다

### 2. 서보 모터 제어

#### 개별 제어
- **슬라이더**: 마우스로 드래그하여 값을 조정 (1000~2000μs)
- **스핀박스**: 숫자를 직접 입력하여 정밀하게 조정
- **프리셋 버튼**: 최소/중앙/최대 값으로 빠르게 설정

#### 통합 제어
- **양쪽 중앙**: 두 서보를 1500μs로 설정 (정지)
- **양쪽 최소**: 두 서보를 1000μs로 설정
- **양쪽 최대**: 두 서보를 2000μs로 설정
- **좌회전**: Servo1=2000, Servo2=1000
- **우회전**: Servo1=1000, Servo2=2000
- **정지**: 두 서보를 1500μs로 설정

### 3. 프로토콜 정보

프로그램은 아두이노에 다음 형식으로 데이터를 전송합니다:

```
# [servo1_high] [servo1_low] [servo2_high] [servo2_low] *
```

- **시작 마커**: `#` (0x23)
- **서보1 펄스 폭**: 2바이트 Big-endian (상위 바이트, 하위 바이트)
- **서보2 펄스 폭**: 2바이트 Big-endian (상위 바이트, 하위 바이트)
- **종료 마커**: `*` (0x2A)

#### 예시
1500μs를 전송하는 경우:
- 1500 = 0x05DC
- 상위 바이트: 0x05
- 하위 바이트: 0xDC
- 양쪽 1500μs: `# 05 DC 05 DC *`

### 4. 로그

- 모든 제어 이벤트와 시리얼 통신 상태가 로그에 기록됩니다
- "로그 지우기" 버튼으로 로그를 삭제할 수 있습니다

## 파일 구조

```
Project/Robotmaster_C620/
├── robotmaster_C620.ino          # 아두이노 펌웨어
├── ui/
│   ├── servo_control.ui          # Qt Designer UI 파일
│   └── README.md                 # UI 설명서
└── scripts/
    ├── servo_control_app.py      # Python 제어 애플리케이션
    ├── requirements.txt          # Python 패키지 의존성
    └── README.md                 # 이 파일
```

## 주요 기능

### ServoControlApp 클래스

#### 메서드
- `refresh_serial_ports()`: 사용 가능한 시리얼 포트 검색
- `connect_serial()`: 시리얼 포트에 연결
- `disconnect_serial()`: 시리얼 포트 연결 해제
- `send_command()`: 서보 제어 명령 전송
- `update_servo1(value)`: 서보1 값 업데이트
- `update_servo2(value)`: 서보2 값 업데이트
- `set_both(servo1_val, servo2_val)`: 두 서보 동시 설정
- `log_message(message)`: 로그 메시지 추가

## 트러블슈팅

### 시리얼 포트가 표시되지 않는 경우

**Linux:**
```bash
# 현재 사용자를 dialout 그룹에 추가
sudo usermod -a -G dialout $USER
# 로그아웃 후 다시 로그인
```

**권한 문제:**
```bash
# 임시로 권한 부여
sudo chmod 666 /dev/ttyUSB0
```

### 연결은 되지만 서보가 동작하지 않는 경우

1. 아두이노에 올바른 펌웨어가 업로드되었는지 확인
2. Baud Rate가 일치하는지 확인 (아두이노: 9600)
3. 시리얼 모니터가 열려있지 않은지 확인 (포트 충돌)
4. 프로토콜 표시창의 데이터가 올바른지 확인

### UI 파일을 찾을 수 없다는 오류

UI 파일 경로가 올바른지 확인:
```python
# servo_control_app.py의 ui_path 변수 확인
ui_path = os.path.join(os.path.dirname(__file__), '..', 'ui', 'servo_control.ui')
```

## 커스터마이징

### 펄스 폭 범위 변경

UI 파일(`servo_control.ui`)에서 슬라이더의 `minimum`과 `maximum` 값을 변경하거나, 코드에서 직접 수정:

```python
self.slider_servo1.setMinimum(500)
self.slider_servo1.setMaximum(2500)
```

### 프로토콜 수정

`send_command()` 메서드에서 프로토콜을 수정할 수 있습니다.

## 라이센스

이 프로젝트는 교육 및 연구 목적으로 자유롭게 사용할 수 있습니다.
