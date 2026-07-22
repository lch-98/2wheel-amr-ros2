// ============================================
// AMR 아두이노 펌웨어 (최종)
// - PID 속도 제어
// - 시리얼 프로토콜 (라즈베리파이 통신)
// - 워치독 (통신 끊김 시 자동 정지)
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

// ── 로봇 스펙 ──────────────────────────────
const float ENCODER_CPR    = 1320.0;
const float WHEEL_DIAMETER = 0.065;
const float WHEEL_CIRCUM   = 3.14159 * WHEEL_DIAMETER;

// ── 피드포워드 ─────────────────────────────
const float FF_SLOPE  = 222.7;
const float FF_OFFSET = 3.4;

// ── PID 게인 ───────────────────────────────
float Kp = 150.0;
float Ki = 300.0;
float Kd = 0.0;

// ── 주기 설정 ──────────────────────────────
const unsigned long CONTROL_PERIOD_MS  = 20;    // 50Hz 제어
const unsigned long FEEDBACK_PERIOD_MS = 20;    // 50Hz 피드백
const unsigned long WATCHDOG_TIMEOUT_MS = 300;  // 300ms 워치독

// ── 안전 제한 ──────────────────────────────
const float MAX_TARGET_VEL = 0.5;

// ── 엔코더 ─────────────────────────────────
volatile long enc_left  = 0;
volatile long enc_right = 0;

// ── 제어 상태 ──────────────────────────────
long prev_enc_left  = 0;
long prev_enc_right = 0;
unsigned long prev_control_time = 0;
unsigned long last_cmd_time = 0;

float target_vel_l = 0.0;
float target_vel_r = 0.0;
float actual_vel_l = 0.0;
float actual_vel_r = 0.0;
float integral_l = 0.0;
float integral_r = 0.0;
float prev_error_l = 0.0;
float prev_error_r = 0.0;

const float INTEGRAL_MAX = 200.0;
const float VEL_FILTER   = 0.6;

bool watchdog_triggered = false;

// ── 시리얼 수신 버퍼 ───────────────────────
char rx_buf[64];
uint8_t rx_idx = 0;

// ============================================
// 엔코더 ISR
// ============================================
void encLeftA() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) enc_left--;
  else enc_left++;
}
void encLeftB() {
  if (digitalRead(ENC_L_A) != digitalRead(ENC_L_B)) enc_left--;
  else enc_left++;
}
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
void setMotorLeft(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) {
    digitalWrite(L_DIR, HIGH);
    analogWrite(L_PWM, pwm);
  } else {
    digitalWrite(L_DIR, LOW);
    analogWrite(L_PWM, -pwm);
  }
}

void setMotorRight(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) {
    digitalWrite(R_DIR, HIGH);
    analogWrite(R_PWM, pwm);
  } else {
    digitalWrite(R_DIR, LOW);
    analogWrite(R_PWM, -pwm);
  }
}

// ============================================
float feedforward(float v) {
  if (v > 0.001)  return FF_SLOPE * v + FF_OFFSET;
  if (v < -0.001) return FF_SLOPE * v - FF_OFFSET;
  return 0.0;
}

// ============================================
// PID 제어 루프
// ============================================
void controlLoop() {
  unsigned long now = millis();
  float dt = (now - prev_control_time) / 1000.0;
  if (dt <= 0.0) return;

  noInterrupts();
  long curr_l = enc_left;
  long curr_r = enc_right;
  interrupts();

  long delta_l = curr_l - prev_enc_left;
  long delta_r = curr_r - prev_enc_right;

  float raw_vel_l = (delta_l / ENCODER_CPR) * WHEEL_CIRCUM / dt;
  float raw_vel_r = (delta_r / ENCODER_CPR) * WHEEL_CIRCUM / dt;

  actual_vel_l = VEL_FILTER * actual_vel_l + (1.0 - VEL_FILTER) * raw_vel_l;
  actual_vel_r = VEL_FILTER * actual_vel_r + (1.0 - VEL_FILTER) * raw_vel_r;

  // 목표 0이면 정지
  if (fabs(target_vel_l) < 0.001 && fabs(target_vel_r) < 0.001) {
    setMotorLeft(0);
    setMotorRight(0);
    integral_l = 0;
    integral_r = 0;
    prev_error_l = 0;
    prev_error_r = 0;
    prev_enc_left  = curr_l;
    prev_enc_right = curr_r;
    prev_control_time = now;
    return;
  }

  float error_l = target_vel_l - actual_vel_l;
  float error_r = target_vel_r - actual_vel_r;

  integral_l += error_l * dt;
  integral_r += error_r * dt;
  integral_l = constrain(integral_l, -INTEGRAL_MAX / Ki, INTEGRAL_MAX / Ki);
  integral_r = constrain(integral_r, -INTEGRAL_MAX / Ki, INTEGRAL_MAX / Ki);

  float d_l = (error_l - prev_error_l) / dt;
  float d_r = (error_r - prev_error_r) / dt;

  float pwm_l = feedforward(target_vel_l) + Kp * error_l + Ki * integral_l + Kd * d_l;
  float pwm_r = feedforward(target_vel_r) + Kp * error_r + Ki * integral_r + Kd * d_r;

  setMotorLeft((int)pwm_l);
  setMotorRight((int)pwm_r);

  prev_error_l = error_l;
  prev_error_r = error_r;
  prev_enc_left  = curr_l;
  prev_enc_right = curr_r;
  prev_control_time = now;
}

// ============================================
// 명령 파싱
//   "v <l> <r>"  속도 명령
//   "s"          정지
//   "r"          엔코더 리셋
//   "p <kp> <ki> <kd>"  게인 변경
// ============================================
void parseCommand(char *buf) {
  if (buf[0] == 'v' || buf[0] == 'V') {
    // 공백으로 토큰 분리
    char *p = buf + 1;
    while (*p == ' ') p++;              // 앞쪽 공백 건너뛰기
    char *sp = strchr(p, ' ');          // 두 값 사이 공백 찾기
    if (sp != NULL) {
      *sp = '\0';                       // 첫 번째 값 끝내기
      float vl = atof(p);
      float vr = atof(sp + 1);

      target_vel_l = constrain(vl, -MAX_TARGET_VEL, MAX_TARGET_VEL);
      target_vel_r = constrain(vr, -MAX_TARGET_VEL, MAX_TARGET_VEL);
      last_cmd_time = millis();
      watchdog_triggered = false;

      Serial.print("# CMD OK: ");
      Serial.print(target_vel_l, 3);
      Serial.print(" ");
      Serial.println(target_vel_r, 3);
    } else {
      Serial.println("# CMD PARSE FAIL");
    }
  }
  else if (buf[0] == 's' || buf[0] == 'S') {
    target_vel_l = 0.0;
    target_vel_r = 0.0;
    last_cmd_time = millis();
    Serial.println("# STOP");
  }
  else if (buf[0] == 'r' || buf[0] == 'R') {
    noInterrupts();
    enc_left = 0;
    enc_right = 0;
    interrupts();
    prev_enc_left = 0;
    prev_enc_right = 0;
    Serial.println("# encoder reset");
  }
  else if (buf[0] == 'p' || buf[0] == 'P') {
    char *p = buf + 1;
    while (*p == ' ') p++;
    char *s1 = strchr(p, ' ');
    if (s1 == NULL) return;
    *s1 = '\0';
    char *s2 = strchr(s1 + 1, ' ');
    if (s2 == NULL) return;
    *s2 = '\0';

    Kp = atof(p);
    Ki = atof(s1 + 1);
    Kd = atof(s2 + 1);
    integral_l = 0;
    integral_r = 0;

    Serial.print("# gains ");
    Serial.print(Kp); Serial.print(" ");
    Serial.print(Ki); Serial.print(" ");
    Serial.println(Kd);
  }
}

// ============================================
// 시리얼 수신 (논블로킹)
// ============================================
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (rx_idx > 0) {
        rx_buf[rx_idx] = '\0';
        parseCommand(rx_buf);
        rx_idx = 0;
      }
    } else {
      if (rx_idx < sizeof(rx_buf) - 1) {
        rx_buf[rx_idx++] = c;
      }
    }
  }
}

// ============================================
// 엔코더 피드백 전송
//   "e <enc_left> <enc_right>"
// ============================================
void sendFeedback() {
  noInterrupts();
  long l = enc_left;
  long r = enc_right;
  interrupts();

  Serial.print("e ");
  Serial.print(l);
  Serial.print(" ");
  Serial.println(r);
}

// ============================================
// 워치독
// ============================================
void checkWatchdog() {
  if (millis() - last_cmd_time > WATCHDOG_TIMEOUT_MS) {
    if (!watchdog_triggered &&
        (fabs(target_vel_l) > 0.001 || fabs(target_vel_r) > 0.001)) {
      target_vel_l = 0.0;
      target_vel_r = 0.0;
      integral_l = 0;
      integral_r = 0;
      setMotorLeft(0);
      setMotorRight(0);
      watchdog_triggered = true;
      Serial.println("# WATCHDOG: stopped");
    }
  }
}

// ============================================
void setup() {
  Serial.begin(115200);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), encLeftA,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), encLeftB,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), encRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), encRightB, CHANGE);

  pinMode(L_DIR, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  setMotorLeft(0);
  setMotorRight(0);

  prev_control_time = millis();
  last_cmd_time = millis();

  Serial.println("# AMR firmware ready");
}

// ============================================
void loop() {
  readSerial();
  checkWatchdog();

  unsigned long now = millis();
  static unsigned long last_control  = 0;
  static unsigned long last_feedback = 0;

  if (now - last_control >= CONTROL_PERIOD_MS) {
    last_control = now;
    controlLoop();
  }

  if (now - last_feedback >= FEEDBACK_PERIOD_MS) {
    last_feedback = now;
    sendFeedback();
  }
}
