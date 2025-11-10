#define NO_DATA  6
unsigned char buf[11];

const int servo1Pin = 2;  // 서보 1 핀
const int servo2Pin = 4;  // 서보 2 핀

union
{
  byte bytedata[2];
  short pulsewidth;
} pwm_command1, pwm_command2;

void setup()
{
  pinMode(servo1Pin, OUTPUT);
  pinMode(servo2Pin, OUTPUT);

  // PWM 초기값 설정 (중립값 1444us)
  pwm_command1.pulsewidth = 1444;
  pwm_command2.pulsewidth = 1444;

  // put your setup code here, to run once:
  Serial1.begin(9600); // Serial Port 1
  Serial.begin(9600);

  // Timer1 설정
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

}

void loop()
{
  // put your main code here, to run repeatedly:

  int read_byte = 0;
  unsigned char in_byte;
  read_byte = Serial1.available();
  //  Serial.print("data in : ");
  //  Serial.println(read_byte);

  if (read_byte > 0)
  {
    Serial.print("data in : ");
    Serial.println(read_byte);

    for (int i = 0; i < NO_DATA - 1; i++)
    {
      buf[i] = buf[i + 1];
    }
    buf[NO_DATA - 1] = Serial1.read();

    if ( (buf[0] == '#') && (buf[NO_DATA - 1] == '*') )
    {
      pwm_command1.bytedata[1] = buf[1];
      pwm_command1.bytedata[0] = buf[2];
      pwm_command2.bytedata[1] = buf[3];
      pwm_command2.bytedata[0] = buf[4];

      // 서보모터 1 (왼쪽) 방향 반전: 2888 - 입력값
      pwm_command1.pulsewidth = 2888 - pwm_command1.pulsewidth;

      Serial.println("Receive Protocol");
      Serial.print("left pulse width : ");
      Serial.println(pwm_command1.pulsewidth);
      Serial.print("right pulse width : ");
      Serial.println(pwm_command2.pulsewidth);

      Serial.print(pwm_command1.bytedata[1], HEX);
      Serial.println(pwm_command1.bytedata[0], HEX);
    }

    for (int i = 0; i < NO_DATA ; i++)
    {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println(" ");
  }
}

// 타이머1 ISR
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
