# DJI RoboMaster C620 캘리브레이션 가이드

## 개요
DJI RoboMaster C620을 PWM 모드로 사용하려면 초기 캘리브레이션이 필요합니다.

## 방법 1: RoboMaster Assistant 소프트웨어 (권장)

### 필요한 것
- RoboMaster Assistant (DJI 공식 소프트웨어)
- USB-CAN 어댑터 또는 DJI 디버거

### 절차
1. RoboMaster Assistant 실행
2. C620을 CAN 버스에 연결
3. 설정 메뉴에서 PWM 모드 활성화
4. PWM 신호 범위 설정 (1000-2000μs)
5. 설정 저장

## 방법 2: Python UI를 통한 수동 캘리브레이션

### 사전 준비
1. 아두이노와 Python UI 연결 완료
2. C620 전원 **OFF** 상태
3. 모터 연결 확인

### 자동 캘리브레이션 사용

#### Python UI에서:
1. **시리얼 포트 연결**
2. 메뉴: `도구` → `C620 캘리브레이션` 클릭
3. 안내에 따라 진행:
   - 최대값(2000μs) 전송 시작
   - C620 전원 ON
   - 비프음 대기
   - 최소값(1000μs) 전송
   - 비프음 확인
4. 완료!

### 아밍(Arming) 기능

캘리브레이션 후 매번 사용 시:
1. 메뉴: `도구` → `C620 아밍` 클릭
2. 3초간 최소값 전송
3. 비프음 확인
4. 제어 시작

## 방법 3: 수동 절차 (Python UI 없이)

### 1단계: 최대값 설정
```arduino
// 아두이노에 업로드할 코드
void setup() {
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
}

void loop() {
  // 2000μs 펄스 생성 (최대값)
  digitalWrite(2, HIGH);
  delayMicroseconds(2000);
  digitalWrite(2, LOW);

  digitalWrite(4, HIGH);
  delayMicroseconds(2000);
  digitalWrite(4, LOW);

  delay(18);  // 20ms 주기
}
```

### 2단계: C620 전원 ON
- 비프음 대기

### 3단계: 최소값 설정
```arduino
void loop() {
  // 1000μs 펄스 생성 (최소값)
  digitalWrite(2, HIGH);
  delayMicroseconds(1000);
  digitalWrite(2, LOW);

  digitalWrite(4, HIGH);
  delayMicroseconds(1000);
  digitalWrite(4, LOW);

  delay(19);  // 20ms 주기
}
```

### 4단계: 비프음 확인
- 캘리브레이션 완료!

## 배선 확인

### 필수 연결
```
Arduino Pin 2 → C620 #1 PWM 입력 (흰색/노란색 선)
Arduino Pin 4 → C620 #2 PWM 입력 (흰색/노란색 선)
Arduino GND  → C620 GND (검은색 선)

C620 전원: 11.1V~22.2V (3S~6S LiPo)
M3508 모터 → C620 모터 출력
```

### 신호 레벨
- 아두이노: 5V 로직
- C620: 3.3V~5V 호환 (대부분의 경우 5V 직접 연결 가능)

## 문제 해결

### 비프음이 들리지 않는 경우
1. **전원 확인**: C620에 충분한 전압(11.1V 이상) 공급
2. **배선 확인**: 신호선과 GND 연결 확인
3. **PWM 신호 확인**: 오실로스코프나 로직 분석기로 확인
4. **주파수 확인**: 50Hz(20ms 주기) 사용

### 모터가 움직이지 않는 경우
1. **캘리브레이션 재실행**
2. **아밍 절차 수행**: 최소값 3초 유지
3. **모터 연결 확인**: 3선 모두 올바르게 연결
4. **C620 LED 상태** 확인:
   - 빨강: 에러
   - 녹색: 정상
   - 깜빡임: 신호 대기 중

### PWM 모드가 작동하지 않는 경우
- 일부 C620 버전은 CAN 모드 전용일 수 있음
- DIP 스위치 또는 점퍼 설정 확인
- 펌웨어 버전 확인 (RoboMaster Assistant 필요)

## 테스트

### 기본 동작 테스트
1. 아밍 완료 후
2. 1500μs (중앙값) → 모터 정지
3. 1600μs → 모터 저속 회전
4. 2000μs → 모터 최대 속도
5. 1000μs → 모터 역회전 (양방향 ESC인 경우)

### Python UI에서 테스트
1. "양쪽 중앙" 클릭 → 정지
2. 슬라이더를 천천히 움직임
3. 모터 반응 확인

## 참고 자료
- DJI RoboMaster 공식 문서
- RoboMaster Assistant 다운로드
- M3508 모터 사양서

## 안전 주의사항
⚠️ **경고**:
- 프로펠러를 연결하지 마세요!
- 모터가 예상치 않게 회전할 수 있습니다
- 캘리브레이션 중 모터에서 멀리 떨어지세요
- 적절한 전원 용량 확인
- 과전류 보호 장치 사용 권장
