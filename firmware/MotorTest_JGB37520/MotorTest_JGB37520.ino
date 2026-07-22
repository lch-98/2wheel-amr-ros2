// ============================================
// MDD10A 모터 구동 테스트
// Arduino Mega 2560
// ============================================

// ── 모터 제어 핀 ───────────────────────────
#define L_DIR  4    // 왼쪽 모터 방향
#define L_PWM  5    // 왼쪽 모터 속도 (PWM)
#define R_PWM  6    // 오른쪽 모터 속도 (PWM)
#define R_DIR  7    // 오른쪽 모터 방향

// ============================================
// 모터 제어 함수
// speed: -255 ~ +255
//   양수 = 전진 방향
//   음수 = 후진 방향
//   0    = 정지
// ============================================

void setMotorLeft(int speed) {
  // 범위 제한
  if (speed > 255)  speed = 255;
  if (speed < -255) speed = -255;

  if (speed >= 0) {
    digitalWrite(L_DIR, HIGH);   // 정방향
    analogWrite(L_PWM, speed);
  } else {
    digitalWrite(L_DIR, LOW);    // 역방향
    analogWrite(L_PWM, -speed);  // 음수를 양수로
  }
}

void setMotorRight(int speed) {
  if (speed > 255)  speed = 255;
  if (speed < -255) speed = -255;

  // ⚠️ 오른쪽 모터는 반대로 장착되어 있으므로
  //    부호를 뒤집어서 "양수 = 전진"으로 통일
  //speed = -speed;

  if (speed >= 0) {
    digitalWrite(R_DIR, HIGH);
    analogWrite(R_PWM, speed);
  } else {
    digitalWrite(R_DIR, LOW);
    analogWrite(R_PWM, -speed);
  }
}

// 두 모터 동시 정지
void stopMotors() {
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}

// ============================================
void setup() {
  Serial.begin(115200);

  pinMode(L_DIR, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  stopMotors();

  Serial.println("=== Motor Test Start ===");
  Serial.println("5 seconds to prepare...");
  delay(5000);   // 준비 시간 (로봇을 들어올릴 시간)
}

// ============================================
void loop() {
  
  // ── 1. 왼쪽 모터만 전진 ──────────────────
  Serial.println("[1] LEFT motor FORWARD");
  setMotorLeft(100);
  setMotorRight(0);
  delay(2000);
  stopMotors();
  delay(1000);

  // ── 2. 왼쪽 모터만 후진 ──────────────────
  Serial.println("[2] LEFT motor BACKWARD");
  setMotorLeft(-100);
  setMotorRight(0);
  delay(2000);
  stopMotors();
  delay(1000);

  // ── 3. 오른쪽 모터만 전진 ────────────────
  Serial.println("[3] RIGHT motor FORWARD");
  setMotorLeft(0);
  setMotorRight(100);
  delay(2000);
  stopMotors();
  delay(1000);

  // ── 4. 오른쪽 모터만 후진 ────────────────
  Serial.println("[4] RIGHT motor BACKWARD");
  setMotorLeft(0);
  setMotorRight(-100);
  delay(2000);
  stopMotors();
  delay(1000);

  // ── 5. 양쪽 전진 (직진) ──────────────────
  Serial.println("[5] BOTH FORWARD (straight)");
  setMotorLeft(100);
  setMotorRight(100);
  delay(2000);
  stopMotors();
  delay(1000);

  // ── 6. 양쪽 후진 ─────────────────────────
  Serial.println("[6] BOTH BACKWARD");
  setMotorLeft(-100);
  setMotorRight(-100);
  delay(2000);
  stopMotors();
  delay(1000);

  // ── 7. 제자리 좌회전 ─────────────────────
  Serial.println("[7] TURN LEFT (in place)");
  setMotorLeft(-100);
  setMotorRight(100);
  delay(2000);
  stopMotors();
  delay(1000);

  // ── 8. 제자리 우회전 ─────────────────────
  Serial.println("[8] TURN RIGHT (in place)");
  setMotorLeft(100);
  setMotorRight(-100);
  delay(2000);
  stopMotors();
  delay(3000);

  Serial.println("=== Cycle complete, repeating ===\n");
}
