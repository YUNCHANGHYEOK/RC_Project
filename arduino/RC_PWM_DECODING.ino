#include <Servo.h>
#include <PinChangeInterrupt.h>

// === 핀 정의 ===
#define CH1        5   // 조향 (RC 방향 채널)
#define CH2        4   // 속도 (RC 스로틀 채널)
#define CH6        6   // 모드 전환 (RC 스위치)
#define MOTOR      9   // ESC 제어 핀
#define DIRECTION 10   // 서보 조향 핀
#define LED_LEFT   7
#define LED_RIGHT  8

// === 기본 설정 ===
const int pwmCenter = 1500;
const float forwardFactor  = 0.25f;
const float backwardFactor = 0.38f;
const int steerRange = 700;
const float steerExpo = 1.5f;

// === 속도 설정 (수정됨) ===
const int straightSpeed = 1545;  // ↓ 느리게 전진
const int cornerSpeed   = 1541;  // ↓ 느리게 회전
const int reverseSpeed  = 1437;  // ↑ 빠르게 후진

// === 라인 거리 판단 기준 ===
const int lineFarThreshold = 160;  // y값 기준 (작을수록 선이 멀다)

// === 시리얼 처리 타이밍 ===
const int serialFrameTimeout = 100;     // 프레임 수신 제한 시간(ms)
const int lostLineThreshold = 1;        // 연속 실패 프레임 수

// === PWM 신호 측정 변수 ===
volatile uint32_t throttleStart, throttlePulse;
volatile uint32_t steerStart, steerPulse;
volatile uint32_t ch6Start, ch6Pulse;

Servo throttleESC;
Servo steerServo;

// === 시리얼 및 상태 변수 ===
String serialBuffer = "";
unsigned long lastSerialTime = 0;
unsigned long lastCH6Time = 0;
int lineCenter = -1;
int lineY = -1;
const int frameCenter = 320;
int lastOffset = 0;
int lostLineCount = 0;

// === 인터럽트 ===
void throttleISR() {
  if (digitalRead(CH2) == HIGH) throttleStart = micros();
  else throttlePulse = micros() - throttleStart;
}

void steerISR() {
  if (digitalRead(CH1) == HIGH) steerStart = micros();
  else steerPulse = micros() - steerStart;
}

void ch7ISR() {
  if (digitalRead(CH6) == HIGH) ch6Start = micros();
  else {
    ch6Pulse = micros() - ch6Start;
    lastCH6Time = millis();
  }
}

// === 초기화 ===
void setup() {
  Serial.begin(9600);

  throttleESC.attach(MOTOR);
  steerServo.attach(DIRECTION);

  delay(2000);  // ESC 초기화 대기
  throttleESC.writeMicroseconds(pwmCenter);
  steerServo.writeMicroseconds(pwmCenter);

  pinMode(CH2, INPUT);
  pinMode(CH1, INPUT);
  pinMode(CH6, INPUT);
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);

  attachPCINT(digitalPinToPCINT(CH2), throttleISR, CHANGE);
  attachPCINT(digitalPinToPCINT(CH1), steerISR, CHANGE);
  attachPCINT(digitalPinToPCINT(CH6), ch7ISR, CHANGE);
}

// === 메인 루프 ===
void loop() {
  // --- 시리얼 입력 처리 ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      int commaIndex = serialBuffer.indexOf(',');
      if (commaIndex > 0) {
        lineCenter = serialBuffer.substring(0, commaIndex).toInt();
        lineY = serialBuffer.substring(commaIndex + 1).toInt();
        lastSerialTime = millis();
        lostLineCount = 0;  // 정상 프레임 들어오면 실패 카운터 초기화
      }
      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }

  // --- 연속 프레임 실패 판정 ---
  if (millis() - lastSerialTime > serialFrameTimeout) {
    if (lostLineCount < lostLineThreshold) {
      lostLineCount++;
      lastSerialTime = millis();  // 다음 프레임 기준 갱신
    }
    if (lostLineCount >= lostLineThreshold) {
      lineCenter = -1;
      lineY = -1;
    }
  }

  if (millis() - lastCH6Time > 1000) ch6Pulse = 0;

  bool useSerial = (ch6Pulse > 1750);
  int steerOut = pwmCenter;
  int throttleOut = pwmCenter;

  if (useSerial) {
    // === 자율주행 모드 ===
    if (lineCenter != -1 && lineY != -1) {
      int offset = frameCenter - lineCenter;
      lastOffset = offset;

      if (abs(offset) <= 20) {
        steerOut = pwmCenter;
      } else {
        float norm = float(offset) / frameCenter;
        float curved = copysign(pow(abs(norm), steerExpo), norm);
        steerOut = pwmCenter + curved * steerRange;
      }

      if (lineY < lineFarThreshold)
        throttleOut = straightSpeed;
      else
        throttleOut = cornerSpeed;

    } else {
      // === 선을 감지하지 못했을 때 후진 ===
      int reverseOffset = 0;
      if (lastOffset > 0) {
        reverseOffset = -300;  // 오른쪽으로 틀다 놓침 → 왼쪽으로 후진
      } else if (lastOffset < 0) {
        reverseOffset = 300;   // 왼쪽으로 틀다 놓침 → 오른쪽으로 후진
      } else {
        reverseOffset = 0;
      }

      float norm = float(reverseOffset) / frameCenter;
      float curved = copysign(pow(abs(norm), steerExpo), norm);
      steerOut = pwmCenter + curved * steerRange;
      throttleOut = reverseSpeed;
    }

    // === 자율주행 LED ===
    if (steerOut < pwmCenter - 100) {
      digitalWrite(LED_LEFT, millis() % 500 < 250);
      digitalWrite(LED_RIGHT, LOW);
    } else if (steerOut > pwmCenter + 100) {
      digitalWrite(LED_RIGHT, millis() % 500 < 250);
      digitalWrite(LED_LEFT, LOW);
    } else {
      digitalWrite(LED_LEFT, LOW);
      digitalWrite(LED_RIGHT, LOW);
    }

    if (throttleOut < pwmCenter - 50) {
      digitalWrite(LED_LEFT, HIGH);
      digitalWrite(LED_RIGHT, HIGH);
    }

  } else {
    // === 수동 모드 ===
    int tOff = throttlePulse - pwmCenter;
    if (tOff > 100)
      throttleOut = pwmCenter + tOff * forwardFactor;
    else if (tOff < -100)
      throttleOut = pwmCenter + tOff * backwardFactor;
    else
      throttleOut = pwmCenter;

    int sOff = steerPulse - pwmCenter;
    if (abs(sOff) > 30) {
      float norm = float(sOff) / 500.0f;
      float curved = copysign(pow(abs(norm), steerExpo), norm);
      steerOut = pwmCenter + curved * steerRange;
    } else {
      steerOut = pwmCenter;
    }

    if (steerOut < pwmCenter - 100) {
      digitalWrite(LED_LEFT, millis() % 500 < 250);
      digitalWrite(LED_RIGHT, LOW);
    } else if (steerOut > pwmCenter + 100) {
      digitalWrite(LED_RIGHT, millis() % 500 < 250);
      digitalWrite(LED_LEFT, LOW);
    } else {
      digitalWrite(LED_LEFT, LOW);
      digitalWrite(LED_RIGHT, LOW);
    }

    if (throttleOut < pwmCenter - 50) {
      digitalWrite(LED_LEFT, HIGH);
      digitalWrite(LED_RIGHT, HIGH);
    }
  }

  // === PWM 출력 ===
  steerOut = constrain(steerOut, 1000, 2000);
  throttleOut = constrain(throttleOut, 1000, 2000);
  steerServo.writeMicroseconds(steerOut);
  throttleESC.writeMicroseconds(throttleOut);

  // === 디버깅 ===
  Serial.print("MODE: ");
  Serial.print(useSerial ? "AUTO" : "RC");
  Serial.print(" | CH7: "); Serial.print(ch6Pulse);
  Serial.print(" | LineCenter: "); Serial.print(lineCenter);
  Serial.print(" | LineY: "); Serial.print(lineY);
  Serial.print(" | Steer: "); Serial.print(steerOut);
  Serial.print(" | Throttle: "); Serial.println(throttleOut);

  delay(20);
}
