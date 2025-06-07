# WebRTC 기반 라인트레이싱 자율주행 RC카


[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.9%2B-blue)](https://www.python.org/)
[![Arduino](https://img.shields.io/badge/Arduino-Uno-green)](https://www.arduino.cc/)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-5-red)](https://www.raspberrypi.com/products/raspberry-pi-5/)


---

## 📌 프로젝트 개요

본 프로젝트는 **Raspberry Pi 5**, **PiCamera2**, 그리고 **Arduino Uno**를 기반으로 한 **WebRTC 실시간 영상 스트리밍** 및 **라인트레이싱 자율주행 RC카**를 구현하는 것을 목표로 합니다.

- `PiCamera2`가 캡처한 실시간 영상을 `OpenCV`로 처리하여 라인을 인식하고, 중심 좌표를 계산합니다.
- 해당 좌표는 시리얼 통신을 통해 Arduino에 전달되며, Arduino는 이를 바탕으로 모터(ESC)와 서보를 정밀하게 제어하여 선을 따라 주행합니다.
- RC 조종기(CH1, CH2, CH6)를 통해 수동 제어 또한 가능하며, CH6 스위치로 자율/수동 모드를 전환할 수 있습니다.
- 향후 초음파 센서, LIDAR 등 다양한 확장을 고려한 **모듈형 설계**가 적용되었습니다.

## 🧑‍💻 팀 구성원

| 이름     | 역할 및 기여 내용 |
|----------|------------------|
| 윤창혁   | 시스템 전체 설계 및 통합<br>PiCamera2 영상처리 알고리즘 구현<br>Arduino PWM 제어 알고리즘 구현<br>자율주행 모드에서의 오프셋 기반 조향 알고리즘 최적화 |
| 심창완   | WebRTC 서버 및 웹 인터페이스 구현<br>라인 이탈 복원/대응 알고리즘 개발<br>실시간 웹소켓 통신 설계 및 연결 안정성 확보<br>aiortc 기반 WebRTC 서버 최적화 및 지연 시간 감소 전략 적용 |


## 🗂️ 프로젝트 파일 구조 - 수정예정

```
.
├── hybrid_manual_auto.ino           # 수동 및 자율 주행 통합 아두이노 코드
├── camera_stream_server.py         # PiCamera2 기반 영상 처리 및 WebRTC 서버
├── README.md                        # 본 문서
├── LICENSE                          # 오픈소스 라이선스 (MIT)
└── assets/
    ├── system-banner.png            # 프로젝트 배너 이미지
    ├── system-architecture.png      # 시스템 흐름도
    └── line_detection_example.png   # 라인 인식 시각화 예시
```

## ⚙️ 주요 기능 및 시스템 아키텍처

### 🔧 전체 시스템 흐름도

```
[RC 조종기 (CH1/CH2/CH6)]         [PiCamera2 + Raspberry Pi]
       │                                 │
       └─ PWM 입력 ──┐              ┌────┘
                    ▼              ▼
        [Arduino Uno]   ◀── Serial 통신 ─── [Line Detection + Offset 계산]
          │        │                              ↑
      Servo     ESC(PWM)                     (Python + OpenCV)
          ▼        ▼
       [RC Car] 자율/수동 주행
```

## 📌 핵심 구성 요소

- **PiCamera2 + Raspberry Pi**: HSV 마스크 기반 라인트레이싱 및 영상 스트리밍
- **Arduino Uno**: PWM 입력 디코딩 및 시리얼 수신 기반 서보/ESC 제어
- **WebRTC 서버**: `aiortc`, `websockets` 기반 실시간 영상 송출
- **LED 피드백**: 조향 방향 및 속도 상태를 시각적으로 표시

## 🧠 이미지 처리 및 제어 로직 (Python)

- 입력 해상도: `640x480 (RGB888)`
- 처리 파이프라인:
  1. HSV 색공간 변환
  2. 검은색 마스킹 및 Morphological Closing
  3. ROI1 (하단) → `cv2.fitLine()` 기반 중심 좌표 계산
  4. ROI2 (상단) → 보조 예측 정보 추출
- 시리얼 포맷: `center_x,center_y\n`
- 미검출 시 `-1,-1\n` 전송, 100ms 이상 간격 유지

## 🤖 Arduino 측 동작 로직

### ▶ PWM 입력 처리

- **CH1**: 조향 (Steering)
- **CH2**: 속도 (Throttle)
- **CH6**: 모드 전환

### ▶ 자율 주행 모드 (CH6 > 1750)

- 시리얼 수신 중심 좌표 기반 offset 계산
- offset → 비선형 보정(steerExpo) → steerRange 반영
- Y 좌표에 따라 속도 전환 (`cornerSpeed` ↔ `straightSpeed`)
- 미검출 시 곡선 후진 로직 수행

### ▶ 수동 주행 모드 (CH6 < 1750)

- PWM 직접 디코딩 및 steerExpo 보정
- 전후진 속도 계수 반영

### ▶ LED 피드백

- 좌우 조향: 좌/우 LED 점멸
- 후진 시: 양쪽 LED 점등

## 🔄 통신 프로토콜 (Serial)

| 항목           | 내용                             |
|----------------|----------------------------------|
| 포맷           | `center_x,center_y\n`            |
| 통신 방향      | Raspberry Pi → Arduino           |
| 전송 간격      | 최소 100ms 이상, 값 변경 시 전송 |
| 미검출 처리     | `-1,-1\n` 전송                   |
| Baudrate       | 9600 bps                         |

## 💻 설치 및 실행 방법

### 🐍 Python WebRTC 서버 (Raspberry Pi 5) --- 이부분을 직접 맞게 수정해줘

1. **환경 설치**
   ```bash
   sudo apt update && sudo apt upgrade -y
   sudo apt install python3-picamera2 python3-opencv python3-serial python3-pip -y
   pip install aiortc websockets av
   ```

2. **카메라 인터페이스 활성화**
   ```bash
   sudo raspi-config
   → Interface Options → Camera → Enable → 재부팅
   ```

3. **서버 실행**
   ```bash
   cd [프로젝트 디렉토리]
   python3 camera_stream_server.py
   ```

4. **접속**
   - WebSocket: `ws://[라즈베리파이 IP]:8765`
   - WebRTC 기반 브라우저 실시간 영상 확인
     


### 🔧 Arduino 코드 업로드 (Arduino Uno)

1. **Arduino IDE 설치**
   - [https://www.arduino.cc/en/software](https://www.arduino.cc/en/software)

2. **라이브러리 설치**
   - `PinChangeInterrupt` (스케치 > 라이브러리 관리에서 설치)

3. **업로드 절차**
   - `hybrid_manual_auto.ino` 열기
   - 보드: Arduino Uno / 포트 설정
   - 컴파일 후 업로드

⚠️ 업로드 전 Serial Monitor 종료 필수

## 🖼️ 시각 자료 예시

- **라인 인식 예시**

  ![](assets/line_detection_example.png)

- **시스템 아키텍처**

  ![](assets/system-architecture.png)
  
---

## 📚 기술 스택 요약

### 하드웨어
- Raspberry Pi 5
- PiCamera2
- Arduino Uno
- RC 수신기 + 조종기
- 서보 모터, ESC

### 소프트웨어
- Python 3.9+
- OpenCV, aiortc, websockets
- Arduino IDE 1.8+
  
---

## 🚧 향후 개선 방향

- **PID 조향 제어**: 곡선 주행 성능 향상 (실제 주행때 한번 트랙에서 벗어났다 돌아오는 현상 발생 -> 이를 해결하기 위해 조절)
- **실시간 Web UI**: 브라우저 상에서 주행 정보 확인 및 수동 조작 기능
- **적응형 라인 색상 감지**: HSV 범위 자동 조정 기능 추가 (빛번짐에도 라인을 잘 디텍싱할 수 있도록 조절)
- **다중 ROI 보정**: 선 단절 구간 예측 대응 향상
- **센서 통합 확장**: 초음파, LIDAR 등 외부 센서 연동 (앞에 장애물이 있다면 이를 인식하고 조절할 수 있도록 발전)
- **성능 최적화 및 속도 증가**: 자원의 효율 개선(속도를 더욱 늘려서 빠른 주행이 가능하도록 조절)
  
---
## 📜 라이선스
[MIT License](LICENSE) — LICENSE 파일 참조
