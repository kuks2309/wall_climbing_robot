#include "src/remote_control.h"

#define NO_DATA  6
unsigned char buf[11];

const int servo1Pin = 2;  // 서보 1 핀
const int servo2Pin = 4;  // 서보 2 핀

union
{
  byte bytedata[2];
  short pulsewidth;
} pwm_command1, pwm_command2;

// SBUS Remote Control
RemoteControl* rc = nullptr;
bool sbus_enabled = true;  // SBUS 활성화
unsigned long last_sbus_update = 0;
#define SBUS_UPDATE_INTERVAL 100  // 100ms (10Hz)

// Control mode
enum ControlMode {
  MODE_SERIAL,  // PC control via Serial1
  MODE_SBUS     // RC control via Serial2 (SBUS)
};
ControlMode control_mode = MODE_SERIAL;

// Drive mode (from SBUS CH6)
uint8_t current_drive_mode = RC_MODE_MANUAL;  // 기본값: Manual
uint8_t previous_drive_mode = RC_MODE_MANUAL;

// 드라이브 모드 문자열 반환 함수
const char* getDriveModeString(uint8_t mode)
{
  switch(mode)
  {
    case RC_MODE_MANUAL:
      return "MANUAL";
    case RC_MODE_SEMI_AUTO:
      return "SEMI_AUTO";
    case RC_MODE_FULL_AUTO:
      return "FULL_AUTO";
    default:
      return "UNKNOWN";
  }
}

void setup()
{
  pinMode(servo1Pin, OUTPUT);
  pinMode(servo2Pin, OUTPUT);

  // PWM 초기값 설정 (중립값 1444us)
  pwm_command1.pulsewidth = 1444;
  pwm_command2.pulsewidth = 1444;

  // Serial0: 디버그 출력 (USB)
  Serial.begin(115200);
  while(!Serial)
  {
    ;
  }
  Serial.println("=== Robotmaster C620 Controller ===");
  Serial.println("Serial0 (USB): Debug output");

  // Serial1: PC 제어 프로토콜 (Pins 18/19)
  Serial1.begin(9600);
  while(!Serial1)
  {
    ;
  }
  Serial.println("Serial1: PC control ready (9600 baud)");

  // Serial2: SBUS 수신기 (Pins 16/17)
  Serial.println("Serial2: Initializing SBUS...");
  rc = new RemoteControl(&Serial2);
  rc->begin();
  Serial.println("Serial2: SBUS ready (100000 baud, inverted)");

  // Timer1 설정 (PWM 생성)
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // CTC 모드 설정
  TCCR1B |= (1 << WGM12);

  // 프리스케일러 = 8
  TCCR1B |= (1 << CS11);

  // 15ms 주기 (30000 = 15ms * 2MHz)
  OCR1A = 30000;

  // 타이머1 비교일치 인터럽트 활성화
  TIMSK1 |= (1 << OCIE1A);

  Serial.println("=== Initialization Complete ===");
  Serial.println("Waiting for control input...");
  Serial.println();
}

void loop()
{
  // SBUS 데이터 업데이트 (100ms 간격)
  if (sbus_enabled && (millis() - last_sbus_update >= SBUS_UPDATE_INTERVAL))
  {
    last_sbus_update = millis();

    // SBUS 업데이트
    bool update_success = rc->update();
    bool is_connected = rc->isConnected();

    // SBUS 연결 상태 디버그
    static unsigned long last_status_debug = 0;
    if (millis() - last_status_debug >= 2000)
    {
      last_status_debug = millis();
      Serial.print("SBUS Connected: ");
      Serial.print(is_connected ? "YES" : "NO");
      Serial.print(" | Control Mode: ");
      Serial.println(control_mode == MODE_SERIAL ? "SERIAL" : "SBUS");
    }

    // SBUS 연결됨
    if (update_success && is_connected)
    {
      // 드라이브 모드 업데이트
      current_drive_mode = rc->getDriveMode();

      // Failsafe 감지: CH0, CH1이 모두 거의 중립값이면 송신기 OFF 상태
      int16_t ch0 = rc->getChannel(0);
      int16_t ch1 = rc->getChannel(1);
      bool is_failsafe = (abs(ch0 - 1000) < 50) && (abs(ch1 - 1011) < 50);

      if (is_failsafe)
      {
        // Failsafe 상태: Serial1 모드로 전환
        static bool failsafe_msg_printed = false;
        if (control_mode != MODE_SERIAL)
        {
          Serial.println("=== Transmitter OFF (Failsafe) ===");
          Serial.println("Switching to Serial1 control mode");
          control_mode = MODE_SERIAL;
          pwm_command1.pulsewidth = 1444;
          pwm_command2.pulsewidth = 1444;
          Serial.println("Waiting for Serial1 commands...");
          failsafe_msg_printed = true;
        }
      }
      else
      {
        // 모드 변경 감지
        if (current_drive_mode != previous_drive_mode)
        {
          Serial.println("=== Drive Mode Changed ===");
          Serial.print("Previous: ");
          Serial.println(getDriveModeString(previous_drive_mode));
          Serial.print("Current: ");
          Serial.println(getDriveModeString(current_drive_mode));
          previous_drive_mode = current_drive_mode;
        }

        // FULL_AUTO 모드에서는 Serial1 제어를 유지
        if (current_drive_mode == RC_MODE_FULL_AUTO)
        {
          control_mode = MODE_SERIAL;  // PC 제어 모드
          // Serial1 명령 유지 (processSbusControl 호출 안 함)
        }
        else
        {
          // MANUAL/SEMI_AUTO 모드에서는 SBUS 제어
          control_mode = MODE_SBUS;
          processSbusControl();
        }
      }
    }
    // SBUS 연결 끊김 또는 업데이트 실패
    else if (!is_connected && control_mode == MODE_SBUS)
    {
      // SBUS 연결 끊김 - Serial1 제어 모드로 전환
      Serial.println("=== SBUS disconnected ===");
      Serial.println("Switching to Serial1 control mode");
      control_mode = MODE_SERIAL;

      // 속도 0으로 초기화 (중립 1444us)
      pwm_command1.pulsewidth = 1444;
      pwm_command2.pulsewidth = 1444;

      Serial.println("Motors stopped (1444us)");
      Serial.println("Waiting for Serial1 commands...");
    }
  }
}

// Serial1 이벤트 핸들러 (PC 제어)
void serialEvent1()
{
  while (Serial1.available())
  {
    // 버퍼 시프트
    for (int i = 0; i < NO_DATA - 1; i++)
    {
      buf[i] = buf[i + 1];
    }
    buf[NO_DATA - 1] = Serial1.read();

    // 프로토콜 확인: # ... *
    if ( (buf[0] == '#') && (buf[NO_DATA - 1] == '*') )
    {
      // MANUAL/SEMI_AUTO 모드에서는 Serial1 명령 무시
      if (control_mode == MODE_SBUS &&
          (current_drive_mode == RC_MODE_MANUAL || current_drive_mode == RC_MODE_SEMI_AUTO))
      {
        // Serial1 명령 무시 (아무것도 하지 않음)
        continue;
      }

      // FULL_AUTO 모드 또는 SBUS 미연결 시에만 Serial1 명령 처리
      pwm_command1.bytedata[1] = buf[1];
      pwm_command1.bytedata[0] = buf[2];
      pwm_command2.bytedata[1] = buf[3];
      pwm_command2.bytedata[0] = buf[4];

      // 서보모터 1 (왼쪽) 방향 반전: 2888 - 입력값
      pwm_command1.pulsewidth = 2888 - pwm_command1.pulsewidth;

      // Serial1 수신 디버그 출력
      Serial.println("=== Serial1 Command Received ===");
      Serial.print("Left PWM: ");
      Serial.println(pwm_command1.pulsewidth);
      Serial.print("Right PWM: ");
      Serial.println(pwm_command2.pulsewidth);
      Serial.print("Control Mode: ");
      Serial.println(control_mode == MODE_SERIAL ? "SERIAL" : "SBUS");
      Serial.flush();
    }

    // 버퍼 디버그 출력 (주석 처리)
    // Serial.print("Buffer: ");
    // for (int i = 0; i < NO_DATA ; i++)
    // {
    //   Serial.print(buf[i], HEX);
    //   Serial.print(" ");
    // }
    // Serial.println(" ");
    // Serial.flush();
  }
}

// SBUS 제어 처리
void processSbusControl()
{
  // SBUS 파라미터 (참조 파일 기준)
  const int16_t SBUS_CENTER = 1011;
  const int16_t SBUS_DEADZONE = 20;
  const int16_t SBUS_MIN = 200;
  const int16_t SBUS_MAX = 1800;
  const int16_t RC_MAX_PWM = 150;

  // SBUS 채널 값 읽기
  int16_t ch0 = rc->getChannel(0);  // 조향 (Aileron)
  int16_t ch1 = rc->getChannel(1);  // 스로틀 (Elevator)

  int16_t base_speed = 0;
  int16_t turn_adjust = 0;
  int16_t left_speed = 0;
  int16_t right_speed = 0;

  // MANUAL / SEMI_AUTO: SBUS 직접 제어 모드
  // (FULL_AUTO는 이 함수를 호출하지 않으므로 여기 없음)

  // CH1 데드존 적용 (중립 위치 안정화)
  if (abs(ch1 - SBUS_CENTER) < SBUS_DEADZONE)
  {
    ch1 = SBUS_CENTER;
  }

  // CH1 스로틀 → 기본 속도 계산
  if (ch1 < SBUS_CENTER)
  {
    // 전진: 200~1011 → +150~0
    base_speed = map(ch1, SBUS_MIN, SBUS_CENTER, RC_MAX_PWM, 0);
  }
  else if (ch1 > SBUS_CENTER)
  {
    // 후진: 1011~1800 → 0~-150
    base_speed = map(ch1, SBUS_CENTER, SBUS_MAX, 0, -RC_MAX_PWM);
  }

  // CH0 조향 → 차동 조정값 계산
  // 중립(1011): 0, 좌측(200): -75, 우측(1800): +75
  turn_adjust = map(ch0, SBUS_MIN, SBUS_MAX, -75, 75);

  // Differential driving 적용
  // 왼쪽 모터: base_speed - turn (우회전 시 감속)
  // 오른쪽 모터: base_speed + turn (좌회전 시 감속)
  left_speed = base_speed - turn_adjust;
  right_speed = base_speed + turn_adjust;

  // 속도 제한
  left_speed = constrain(left_speed, -RC_MAX_PWM, RC_MAX_PWM);
  right_speed = constrain(right_speed, -RC_MAX_PWM, RC_MAX_PWM);

  // PWM 속도(-150~+150) → C620 펄스폭(1000~2000us) 변환
  // 중립 1444us 기준: -150 → 1000us, 0 → 1444us, +150 → 1888us
  int16_t left_pwm = map(left_speed, -RC_MAX_PWM, RC_MAX_PWM, 1000, 1888);
  int16_t right_pwm = map(right_speed, -RC_MAX_PWM, RC_MAX_PWM, 1000, 1888);

  // 모터 1 (왼쪽)은 방향 반전 필요
  pwm_command1.pulsewidth = 2888 - left_pwm;
  pwm_command2.pulsewidth = right_pwm;

  // 디버그 출력 (1초에 한 번)
  static unsigned long last_debug = 0;
  if (millis() - last_debug >= 1000)
  {
    last_debug = millis();

    // PWM 값을 -100~100% 범위로 변환
    // left_speed, right_speed는 -150~150 범위
    int16_t left_percent = map(left_speed, -RC_MAX_PWM, RC_MAX_PWM, -100, 100);
    int16_t right_percent = map(right_speed, -RC_MAX_PWM, RC_MAX_PWM, -100, 100);

    Serial.println("=== SBUS Control ===");
    Serial.print("Mode: ");
    Serial.println(getDriveModeString(current_drive_mode));
    Serial.print("CH0: ");
    Serial.print(ch0);
    Serial.print(" | CH1: ");
    Serial.println(ch1);
    Serial.print("Left: ");
    Serial.print(left_percent);
    Serial.print("% | Right: ");
    Serial.print(right_percent);
    Serial.println("%");
    Serial.flush();
  }
}

// 타이머1 ISR (PWM 신호 생성)
ISR(TIMER1_COMPA_vect) {
  static uint8_t state = 0;
  static uint16_t count = 0;

  switch(state) {
    case 0:  // 서보1 시작
      digitalWrite(servo1Pin, HIGH);
      OCR1A = pwm_command1.pulsewidth * 2;  // 펄스 폭 설정
      state = 1;
      break;

    case 1:  // 서보1 종료
      digitalWrite(servo1Pin, LOW);
      OCR1A = 1000;  // 0.5ms 대기
      state = 2;
      break;

    case 2:  // 서보2 시작
      digitalWrite(servo2Pin, HIGH);
      OCR1A = pwm_command2.pulsewidth * 2;
      state = 3;
      break;

    case 3:  // 서보2 종료 및 남은 주기
      digitalWrite(servo2Pin, LOW);
      // 남은 시간 계산: 15ms - (서보1 펄스 + 0.5ms + 서보2 펄스)
      OCR1A = 30000 - (pwm_command1.pulsewidth + pwm_command2.pulsewidth) * 2 - 1000;
      state = 0;
      break;
  }
}
