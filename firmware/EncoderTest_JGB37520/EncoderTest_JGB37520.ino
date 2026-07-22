// ============================================
// JGB37-520 엔코더 읽기 테스트 (핀 교차 보정)
// Arduino Mega 2560
// ============================================

// ── 핀 정의 (교차 보정됨) ──────────────────
#define ENC_L_A  18    // 왼쪽 엔코더 A상
#define ENC_L_B  19    // 왼쪽 엔코더 B상
#define ENC_R_A  2     // 오른쪽 엔코더 A상
#define ENC_R_B  3     // 오른쪽 엔코더 B상

// ── 엔코더 카운트 ──────────────────────────
volatile long enc_left  = 0;
volatile long enc_right = 0;

// ── 모터/바퀴 스펙 ─────────────────────────
const float ENCODER_CPR    = 1320.0;
const float WHEEL_DIAMETER = 0.065;
const float WHEEL_CIRCUM   = 3.14159 * WHEEL_DIAMETER;

// ============================================
// 인터럽트 서비스 루틴 (ISR)
// ── 왼쪽(18,19번): 부호 반전 적용 ──────────
// ============================================
void encLeftA() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) {
    enc_left--;      // 반전
  } else {
    enc_left++;      // 반전
  }
}

void encLeftB() {
  if (digitalRead(ENC_L_A) != digitalRead(ENC_L_B)) {
    enc_left--;      // 반전
  } else {
    enc_left++;      // 반전
  }
}

// ── 오른쪽(2,3번): 부호 그대로 ─────────────
void encRightA() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) {
    enc_right++;
  } else {
    enc_right--;
  }
}

void encRightB() {
  if (digitalRead(ENC_R_A) != digitalRead(ENC_R_B)) {
    enc_right++;
  } else {
    enc_right--;
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

  Serial.println("=== Encoder Test (Pin Swapped) ===");
  Serial.println("Push robot FORWARD by hand!");
  Serial.println("Expect: both L and R increase (positive)");
  Serial.println();
}

// ============================================
void loop() {
  noInterrupts();
  long l = enc_left;
  long r = enc_right;
  interrupts();

  float rev_l = l / ENCODER_CPR;
  float rev_r = r / ENCODER_CPR;
  float dist_l = rev_l * WHEEL_CIRCUM;
  float dist_r = rev_r * WHEEL_CIRCUM;

  Serial.print("L: ");
  Serial.print(l);
  Serial.print(" (");
  Serial.print(rev_l, 2);
  Serial.print(" rev, ");
  Serial.print(dist_l, 3);
  Serial.print(" m)");

  Serial.print("   |   R: ");
  Serial.print(r);
  Serial.print(" (");
  Serial.print(rev_r, 2);
  Serial.print(" rev, ");
  Serial.print(dist_r, 3);
  Serial.println(" m)");

  delay(200);
}
