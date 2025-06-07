#include <Servo.h>                         // 서보 모터 제어를 위한 Servo 라이브러리 포함
#include <PinChangeInterrupt.h>            // 모든 디지털 핀에서 인터럽트 사용을 위한 라이브러리 포함

// === 핀 정의 ===
#define CH1        5   // RC 조종기의 조향 신호 입력 핀
#define CH2        4   // RC 조종기의 스로틀(속도) 신호 입력 핀
#define CH6        6   // RC 조종기의 모드 전환 스위치 입력 핀
#define MOTOR      9   // ESC 제어를 위한 PWM 출력 핀 (스로틀)
#define DIRECTION 10   // 서보 모터 제어를 위한 PWM 출력 핀 (조향)
#define LED_LEFT   7   // 좌측 방향 LED 출력 핀
#define LED_RIGHT  8   // 우측 방향 LED 출력 핀

// === 기본 설정 ===
const int pwmCenter = 1500;             // PWM 중앙값 (중립값)
const float forwardFactor  = 0.25f;     // 전진 시 입력 PWM에 대한 민감도 계수
const float backwardFactor = 0.38f;     // 후진 시 입력 PWM에 대한 민감도 계수
const int steerRange = 700;             // 조향값 최대 편차 (출력 범위)
const float steerExpo = 1.5f;           // 조향 반응의 비선형 지수 계수

// === 속도 설정 (수정됨) ===
const int straightSpeed = 1545;         // 직진 속도 PWM (느리게 전진)
const int cornerSpeed   = 1541;         // 회전 시 속도 PWM (더 느리게 회전)
const int reverseSpeed  = 1437;         // 후진 속도 PWM (빠르게 후진)

// === 라인 거리 판단 기준 ===
const int lineFarThreshold = 160;       // 카메라로 본 선의 y값 기준: 작을수록 멀리 있는 선

// === 시리얼 처리 타이밍 ===
const int serialFrameTimeout = 100;     // 시리얼 데이터 프레임 수신 제한 시간(ms)
const int lostLineThreshold = 1;        // 선 감지 실패를 몇 프레임까지 허용할지 기준

// === PWM 신호 측정 변수 ===
volatile uint32_t throttleStart, throttlePulse;  // RC 스로틀 신호 측정용 변수
volatile uint32_t steerStart, steerPulse;        // RC 조향 신호 측정용 변수
volatile uint32_t ch6Start, ch6Pulse;            // RC 모드 전환 스위치 신호 측정용 변수

Servo throttleESC;                     // ESC 제어용 서보 객체
Servo steerServo;                      // 조향 서보 객체

// === 시리얼 및 상태 변수 ===
String serialBuffer = "";              // 시리얼로 받은 문자열 버퍼
unsigned long lastSerialTime = 0;      // 마지막 시리얼 수신 시각
unsigned long lastCH6Time = 0;         // 마지막 CH6 수신 시각
int lineCenter = -1;                   // 카메라로 감지된 선의 중심 x좌표
int lineY = -1;                        // 카메라로 감지된 선의 y좌표
const int frameCenter = 320;           // 카메라 화면의 중심 x좌표
int lastOffset = 0;                    // 마지막 프레임에서 선 중심의 오프셋
int lostLineCount = 0;                 // 선을 감지하지 못한 연속 프레임 수

// === 인터럽트 ===
void throttleISR() {
  if (digitalRead(CH2) == HIGH) throttleStart = micros();    // 상승 엣지: 시작 시간 저장
  else throttlePulse = micros() - throttleStart;             // 하강 엣지: 펄스 길이 계산
}

void steerISR() {
  if (digitalRead(CH1) == HIGH) steerStart = micros();       // 상승 엣지: 시작 시간 저장
  else steerPulse = micros() - steerStart;                   // 하강 엣지: 펄스 길이 계산
}

void ch7ISR() {
  if (digitalRead(CH6) == HIGH) ch6Start = micros();         // 상승 엣지: 시작 시간 저장
  else {
    ch6Pulse = micros() - ch6Start;                          // 하강 엣지: 펄스 길이 계산
    lastCH6Time = millis();                                  // 마지막 수신 시간 갱신
  }
}

// === 초기화 ===
void setup() {
  Serial.begin(9600);                                       // 시리얼 통신 초기화

  throttleESC.attach(MOTOR);                                // ESC 연결
  steerServo.attach(DIRECTION);                             // 서보 연결

  delay(2000);                                               // ESC 초기화 대기 시간
  throttleESC.writeMicroseconds(pwmCenter);                 // 초기 PWM 값 설정 (정지 상태)
  steerServo.writeMicroseconds(pwmCenter);

  pinMode(CH2, INPUT); pinMode(CH1, INPUT); pinMode(CH6, INPUT);      // 입력 핀 설정
  pinMode(LED_LEFT, OUTPUT); pinMode(LED_RIGHT, OUTPUT);             // LED 출력 핀 설정

  attachPCINT(digitalPinToPCINT(CH2), throttleISR, CHANGE);          // CH2 인터럽트 설정
  attachPCINT(digitalPinToPCINT(CH1), steerISR, CHANGE);             // CH1 인터럽트 설정
  attachPCINT(digitalPinToPCINT(CH6), ch7ISR, CHANGE);               // CH6 인터럽트 설정
}

// === 메인 루프 ===
void loop() {
  // --- 시리얼 입력 처리 ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      int commaIndex = serialBuffer.indexOf(',');
      if (commaIndex > 0) {
        lineCenter = serialBuffer.substring(0, commaIndex).toInt();  // 선 중심 x좌표 추출
        lineY = serialBuffer.substring(commaIndex + 1).toInt();      // 선 y좌표 추출
        lastSerialTime = millis();                                   // 시간 기록
        lostLineCount = 0;                                           // 실패 카운터 초기화
      }
      serialBuffer = "";     // 버퍼 초기화
    } else {
      serialBuffer += c;    // 버퍼에 문자 추가
    }
  }

  // --- 선 감지 실패 판정 ---
  if (millis() - lastSerialTime > serialFrameTimeout) {
    if (lostLineCount < lostLineThreshold) {
      lostLineCount++;
      lastSerialTime = millis();
    }
    if (lostLineCount >= lostLineThreshold) {
      lineCenter = -1;
      lineY = -1;   // 감지 실패 처리
    }
  }

  if (millis() - lastCH6Time > 1000) ch6Pulse = 0;  // 모드 스위치 입력 유효시간 초과 시 초기화

  bool useSerial = (ch6Pulse > 1750);               // 모드 스위치 상태에 따라 자율주행 모드 여부 판단
  int steerOut = pwmCenter;
  int throttleOut = pwmCenter;

  if (useSerial) {
    // === 자율주행 모드 ===
    if (lineCenter != -1 && lineY != -1) {
      int offset = frameCenter - lineCenter;       // 선 중심과 프레임 중심 간 차이 계산
      lastOffset = offset;

      if (abs(offset) <= 20) {
        steerOut = pwmCenter;                     // 중심이면 직진
      } else {
        float norm = float(offset) / frameCenter;         // 오프셋 정규화
        float curved = copysign(pow(abs(norm), steerExpo), norm);  // 비선형 조향값 계산
        steerOut = pwmCenter + curved * steerRange;
      }

      if (lineY < lineFarThreshold)
        throttleOut = straightSpeed;              // 선이 멀면 직진
      else
        throttleOut = cornerSpeed;               // 선이 가까우면 회전 (감속)

    } else {
      // === 선 감지 실패 시 후진 ===
      int reverseOffset = 0;
      if (lastOffset > 0) reverseOffset = -300;    // 오른쪽 놓침 시 왼쪽으로 후진
      else if (lastOffset < 0) reverseOffset = 300; // 왼쪽 놓침 시 오른쪽으로 후진

      float norm = float(reverseOffset) / frameCenter;
      float curved = copysign(pow(abs(norm), steerExpo), norm);
      steerOut = pwmCenter + curved * steerRange;
      throttleOut = reverseSpeed;
    }

    // === 방향 LED 처리 ===
    if (steerOut < pwmCenter - 100) {
      digitalWrite(LED_LEFT, millis() % 500 < 250);  // 좌회전 깜빡이
      digitalWrite(LED_RIGHT, LOW);
    } else if (steerOut > pwmCenter + 100) {
      digitalWrite(LED_RIGHT, millis() % 500 < 250); // 우회전 깜빡이
      digitalWrite(LED_LEFT, LOW);
    } else {
      digitalWrite(LED_LEFT, LOW); digitalWrite(LED_RIGHT, LOW);
    }

    if (throttleOut < pwmCenter - 50) {
      digitalWrite(LED_LEFT, HIGH); digitalWrite(LED_RIGHT, HIGH); // 후진 등
    }

  } else {
    // === 수동 조종 모드 ===
    int tOff = throttlePulse - pwmCenter;
    if (tOff > 100)
      throttleOut = pwmCenter + tOff * forwardFactor;
    else if (tOff < -100)
      throttleOut = pwmCenter + tOff * backwardFactor;
    else
      throttleOut = pwmCenter;  // 중립값

    int sOff = steerPulse - pwmCenter;
    if (abs(sOff) > 30) {
      float norm = float(sOff) / 500.0f;
      float curved = copysign(pow(abs(norm), steerExpo), norm);
      steerOut = pwmCenter + curved * steerRange;
    } else {
      steerOut = pwmCenter;  // 중립값
    }

    // 방향 LED 처리
    if (steerOut < pwmCenter - 100) {
      digitalWrite(LED_LEFT, millis() % 500 < 250);
      digitalWrite(LED_RIGHT, LOW);
    } else if (steerOut > pwmCenter + 100) {
      digitalWrite(LED_RIGHT, millis() % 500 < 250);
      digitalWrite(LED_LEFT, LOW);
    } else {
      digitalWrite(LED_LEFT, LOW); digitalWrite(LED_RIGHT, LOW);
    }

    if (throttleOut < pwmCenter - 50) {
      digitalWrite(LED_LEFT, HIGH); digitalWrite(LED_RIGHT, HIGH);
    }
  }

  // === PWM 출력 ===
  steerOut = constrain(steerOut, 1000, 2000);         // 출력 범위 제한
  throttleOut = constrain(throttleOut, 1000, 2000);
  steerServo.writeMicroseconds(steerOut);             // PWM 출력 (조향)
  throttleESC.writeMicroseconds(throttleOut);         // PWM 출력 (속도)

  // === 디버깅 정보 출력 ===
  Serial.print("MODE: ");
  Serial.print(useSerial ? "AUTO" : "RC");
  Serial.print(" | CH7: "); Serial.print(ch6Pulse);
  Serial.print(" | LineCenter: "); Serial.print(lineCenter);
  Serial.print(" | LineY: "); Serial.print(lineY);
  Serial.print(" | Steer: "); Serial.print(steerOut);
  Serial.print(" | Throttle: "); Serial.println(throttleOut);

  delay(20);  // 주기 조절 (50Hz)
}
