// ============================================
// 엔코더 + 모터 통합 테스트
// PWM 값에 따른 실제 속도 측정
// Arduino Mega 2560
// ============================================

// ── 엔코더 핀 ──────────────────────────────
#define ENC_L_A  18
#define ENC_L_B  19
#define ENC_R_A  2
#define ENC_R_B  3

// ── 모터 제어 핀 ───────────────────────────
#define L_DIR  4
#define L_PWM  5
#define R_PWM  6
#define R_DIR  7

// ── 엔코더 카운트 ──────────────────────────
volatile long enc_left  = 0;
volatile long enc_right = 0;

// ── 로봇 스펙 ──────────────────────────────
const float ENCODER_CPR    = 1320.0;
const float WHEEL_DIAMETER = 0.065;
const float WHEEL_CIRCUM   = 3.14159 * WHEEL_DIAMETER;  // 0.2042 m

// ── 속도 계산용 변수 ───────────────────────
long prev_enc_left  = 0;
long prev_enc_right = 0;
unsigned long prev_time = 0;

// ============================================
// 엔코더 ISR
// ============================================
// 왼쪽 (18, 19번 핀) — 부호 반전
void encLeftA() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) enc_left--;
  else enc_left++;
}
void encLeftB() {
  if (digitalRead(ENC_L_A) != digitalRead(ENC_L_B)) enc_left--;
  else enc_left++;
}

// 오른쪽 (2, 3번 핀) — 부호 그대로
void encRightA() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) enc_right++;
  else enc_right--;
}
void encRightB() {
  if (digitalRead(ENC_R_A) != digitalRead(ENC_R_B)) enc_right++;
  else enc_right--;
}

// ============================================
// 모터 제어
// ============================================
void setMotorLeft(int speed) {
  if (speed > 255)  speed = 255;
  if (speed < -255) speed = -255;

  if (speed >= 0) {
    digitalWrite(L_DIR, HIGH);
    analogWrite(L_PWM, speed);
  } else {
    digitalWrite(L_DIR, LOW);
    analogWrite(L_PWM, -speed);
  }
}

void setMotorRight(int speed) {
  if (speed > 255)  speed = 255;
  if (speed < -255) speed = -255;

  if (speed >= 0) {
    digitalWrite(R_DIR, HIGH);
    analogWrite(R_PWM, speed);
  } else {
    digitalWrite(R_DIR, LOW);
    analogWrite(R_PWM, -speed);
  }
}

void stopMotors() {
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}

// ============================================
// 속도 측정 및 출력
// ============================================
void measureAndPrint(int pwm) {
  // 엔코더 값 안전하게 읽기
  noInterrupts();
  long curr_l = enc_left;
  long curr_r = enc_right;
  interrupts();

  unsigned long curr_time = millis();

  // 경과 시간 (초)
  float dt = (curr_time - prev_time) / 1000.0;
  if (dt <= 0) return;

  // 이 구간 동안의 카운트 변화
  long delta_l = curr_l - prev_enc_left;
  long delta_r = curr_r - prev_enc_right;

  // 회전수 → 이동거리 → 속도
  float dist_l = (delta_l / ENCODER_CPR) * WHEEL_CIRCUM;  // m
  float dist_r = (delta_r / ENCODER_CPR) * WHEEL_CIRCUM;  // m

  float vel_l = dist_l / dt;   // m/s
  float vel_r = dist_r / dt;   // m/s
  // > 속도 = (카운트변화 ÷ 1320) × (π × 0.065) ÷ 시간
  
  // RPM 계산 (바퀴 기준)
  float rpm_l = (delta_l / ENCODER_CPR) / dt * 60.0;
  float rpm_r = (delta_r / ENCODER_CPR) / dt * 60.0;
  // RPM = Revolutions Per Minute (분당 회전수)
  
  // 출력
  Serial.print("PWM: ");
  Serial.print(pwm);
  Serial.print("  |  L: ");
  Serial.print(vel_l, 3);
  Serial.print(" m/s (");
  Serial.print(rpm_l, 1);
  Serial.print(" rpm)");
  Serial.print("  |  R: ");
  Serial.print(vel_r, 3);
  Serial.print(" m/s (");
  Serial.print(rpm_r, 1);
  Serial.println(" rpm)");

  // 다음 계산을 위해 저장
  prev_enc_left  = curr_l;
  prev_enc_right = curr_r;
  prev_time = curr_time;
}

// ============================================
void setup() {
  Serial.begin(115200);

  // 엔코더 설정
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), encLeftA,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), encLeftB,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), encRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), encRightB, CHANGE);

  // 모터 설정
  pinMode(L_DIR, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  stopMotors();

  Serial.println("=== Encoder + Motor Integration Test ===");
  Serial.println("Measuring speed at different PWM values");
  Serial.println();
  delay(3000);

  prev_time = millis();
}

// ============================================
void loop() {
  // 테스트할 PWM 값들
  int pwm_list[] = {10, 15, 20, 25, 30, 40, 50};          // 저속 = Dead Zone 테스트(Dead Zone 아래면 모터가 안 돎)
  // int pwm_list[] = {50, 80, 100, 130, 160, 200, 255};  // 고속 = 왼쪽, 오른쪽 모터 데이터를 확인(PID 제어를 위한 데이터 확보)
  int num_pwm = 7;

  for (int i = 0; i < num_pwm; i++) {
    int pwm = pwm_list[i];

    Serial.print("\n--- Testing PWM = ");
    Serial.print(pwm);
    Serial.println(" ---");

    // 모터 시작
    setMotorLeft(pwm);
    setMotorRight(pwm);

    // 속도가 안정될 때까지 대기
    delay(1000);

    // 기준점 리셋
    noInterrupts();
    prev_enc_left  = enc_left;
    prev_enc_right = enc_right;
    interrupts();
    prev_time = millis();

    // 2초간 0.5초마다 측정
    for (int j = 0; j < 4; j++) {
      delay(500);
      measureAndPrint(pwm);
    }

    // 정지
    stopMotors();
    delay(1500);
  }

  Serial.println("\n=== Test cycle complete ===");
  Serial.println("Repeating in 5 seconds...\n");
  delay(5000);
}
