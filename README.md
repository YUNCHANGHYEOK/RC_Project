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


## 🗂️ 프로젝트 파일 구조

```
.         
├── arduino/
│   └── hybrid_manual_auto.ino       # 수동 및 자율 주행 통합 아두이노 코드
├── README.md                        # 본 문서
├── LICENSE                          # 오픈소스 라이선스 (MIT)
└── WebRTC/
│   ├── camera_stream_server.py      # python WebRTC 서버 코드
│   └── index.html                   # WebRTC 웹 뷰어 코드
└── images/
    └── linetracking.png             # 웹 뷰어 이미지
```

## ⚙️ 주요 기능 및 시스템 아키텍처

### 🔧 전체 시스템 흐름도

```
[RC 조종기 (CH1/CH2/CH6)]         [PiCamera2 + Raspberry Pi]
       │                                 │
       └─ PWM 입력 ──┐              ┌────┘
                    ▼              ▼
        [Arduino Uno]   ◀── Serial 통신 ─── [Line Detection + Offset 계산]
          │        │                              │
      Servo     ESC(PWM)                     (Python + OpenCV)
          ▼        ▼                              │
       [RC Car] 자율/수동 주행                   ▼
                                          [📄 HTML 뷰어 (index.html)]
                                               │
                                  브라우저에서 WebRTC + WebSocket 수신
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
---
# 🔄 Raspberry Pi ↔ Arduino Serial 통신 프로토콜

본 시스템은 **Raspberry Pi (Python)** 에서 영상 처리된 라인트레이싱 결과를 **Arduino (C++)** 로 전송하여 자율주행 동작을 수행합니다.  

  **단방향 Serial 통신(RPi → Arduino)** 기반으로 구현되며, 통신 구조는 단순하지만 실시간성과 안정성을 동시에 제공합니다.

---

### 📬 통신 구조

| 항목             | 설명 |
|------------------|------|
| **통신 방향**     | Raspberry Pi → Arduino (단방향) |
| **통신 속도**     | 9600 bps |
| **전송 주기**     | 최소 100ms 이상<br>(버퍼 오버플로 방지 목적) |
| **데이터 포맷**   | `"center_x,center_y\n"` |
| **정상 예시**     | `320,410\n` |
| **미검출 예시**   | `-1,-1\n` |
| **에러 처리**     | 100ms 이상 수신 없음 또는 `-1,-1` 수신 시 감지 실패로 간주 |

---

### 📤 Raspberry Pi 측 (Python) — 송신 코드

```python
# 중심 좌표가 감지되었을 경우
serial.write(f"{cx},{cy}\n".encode())

# 라인을 찾지 못했을 경우
serial.write("-1,-1\n".encode())
```

- `cx`, `cy`: 라인 중심 좌표 (정수형)
- `\n`: 각 프레임의 종료를 나타내는 **종단 문자**
- 전송 간격: **최소 100ms 이상**

---

### 📥 Arduino 측 (C++) — 수신 코드

```cpp
while (Serial.available()) {
  char c = Serial.read();
  if (c == '\n') {
    int commaIndex = serialBuffer.indexOf(',');
    if (commaIndex > 0) {
      lineCenter = serialBuffer.substring(0, commaIndex).toInt();
      lineY = serialBuffer.substring(commaIndex + 1).toInt();
      lastSerialTime = millis();
      lostLineCount = 0;
    }
    serialBuffer = "";
  } else {
    serialBuffer += c;
  }
}
```

### 🔍 처리 방식

- **문자 단위로 읽기** → `\n` 수신 시 패킷 완성
- `,`를 기준으로 좌표 분리 → `toInt()`로 변환
- 정상 좌표 수신 시 `lastSerialTime` 초기화 및 로직 실행
- 비정상 또는 누락 시 감지 실패 처리

---

### ⚠️ 수신 타임아웃 및 예외 처리

```cpp
if (millis() - lastSerialTime > serialFrameTimeout) {
  if (lostLineCount < lostLineThreshold) {
    lostLineCount++;
    lastSerialTime = millis();
  }
  if (lostLineCount >= lostLineThreshold) {
    lineCenter = -1;
    lineY = -1; // 감지 실패로 간주
  }
}
```

- 일정 시간 수신 없음 시: **`lostLineCount` 증가**
- 연속 실패(`lostLineThreshold` 초과) → **`-1,-1`로 강제 처리**
- 이 상태에서는 **후진 또는 회피 등 안전 로직 실행 가능**

---

### ✅ 요약

| 항목 | 내용 |
|------|------|
| **전송 포맷** | `"center_x,center_y\n"` (UTF-8 인코딩) |
| **전송 주기** | 100ms 이상 |
| **수신 방식** | 문자 단위 파싱, 종단문자 `\n` 기준 |
| **에러 처리** | `-1,-1` 전송 또는 timeout 발생 시 감지 실패 간주 |

---


## 💻 설치방법

### 🐍 Python WebRTC 서버 (Raspberry Pi 5)

1. **프로젝트 클론**
   ```bash
   git clone https://github.com/YUNCHANGHYEOK/RC_Project.git
   cd RC_Project
   ```

2. **가상환경 생성 및 활성화**
   ```bash
   가상환경 생성 및 활성화
   python3 -m venv venv
   source venv/bin/activate  # Windows의 경우: venv\\Scripts\\activate
   ```

3. **필요한 패키지 설치**
   ```bash
   pip install opencv-python aiortc websockets picamera2 numpy
   ```

---

## ▶️ 실행 방법

1. **WebRTC 영상 처리 서버 실행**
   ```bash
   source venv/bin/activate            # 가상환경 활성화
   python camera_stream_server.py     # 라인트레이싱 + WebRTC 서버 실행
   ```
   - Picamera2로 라인을 인식하고
   - 라인의 중심좌표를 아두이노로 전송
   - WebRTC + WebSocket을 통해 브라우저로 영상 스트리밍

2. **클라이언트 HTML 뷰어 실행**
   - 다른 터미널에서 동일한 가상환경을 활성화한 후, HTML 파일이 있는 디렉토리에서 아래 명령어 실행:
   ```bash
   source venv/bin/activate            # 가상환경 활성화
   cd html/                            # (예: HTML 파일이 html 폴더에 있다면)
   python -m http.server 8000         # 웹서버 실행
   ```
   - HTML 내부에 포함된 JavaScript가 WebSocket (8765)을 통해 라인 중심 좌표와 영상을 수신

---
## 🧠 주요 로직 설명 (Python 코드) + 사진

### 📌 초기 버전들

| 초기 버전 1 | 초기 버전 2 | 초기 버전 3 |
|:----:|:----:|:----:|
| ![UI1](https://github.com/user-attachments/assets/75700ac9-ecf4-4bc4-88ab-c4e570896909) | ![UI2](https://github.com/user-attachments/assets/71211712-eb0b-40d5-b1de-949f65bc694d) | ![UI3](https://github.com/user-attachments/assets/225c180f-59cb-4b49-899b-3b2d01c03b30) |

---

#### ✅ 최종 버전

| 최종 버전 |
|:----:|
| ![최종버전](https://github.com/user-attachments/assets/1c693819-6eaf-4cc7-acd8-1a6a0e4db12e) |
> #### 설명
> 
> - 이미지에는 두 개의 초록색 사각형이 보이며, 이는 각각 ROI1 (하단)과 ROI2 (중간 상단)으로 선 추적을 위한 분석 구간입니다.  
> - **노란 점**은 ROI1 내에서 검정 선의 중심을 계산한 좌표로, 차량 제어에 활용됩니다.  
> - **빨간 점**은 ROI2 내 중심으로, 주로 선의 진행 방향 예측이나 조향 판단에 참고됩니다.  
> - 각 ROI의 픽셀 데이터를 기반으로 직선을 추정하고, 중심 좌표를 아두이노 및 WebSocket으로 전송하여 제어 시스템과 연동됩니다.





1. **🎯 HSV 마스크 기반 라인 검출**
   ```python
   hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

   lower_black = np.array([0, 0, 0])
   upper_black = np.array([180, 90, 170])
   mask = cv2.inRange(hsv, lower_black, upper_black)
   ```
   - 카메라 영상에서 어두운(검정색) 선을 인식하기 위해 BGR 이미지를 HSV로 변환한 후, 특정 범위에 해당하는 픽셀만 흰색으로 마스킹

2. **🟩 ROI1에서 중심 좌표 계산(라인 추적)**
   ```python
   roi = mask[320:400, :]  # 하단 영역 잘라내기
   ys, xs = np.where(roi == 255)

   if len(xs) > 100:
       points = np.array(list(zip(xs, ys)))
       [vx, vy, x0, y0] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
       t = ((400 - 320) // 2 - y0) / vy
       cx = int(x0 + vx * t)
   ```
   - 영상 하단 영역(ROI1)에서 흰색  픽셀 위치를 수집한 뒤, 직선을 피팅하여 라인의 중심좌표를 계산
   - 이 값은 RC카의 주행 중심에 활용

3. **🔁 이전 중심 유지 로직(라인 미탐지 대비)**
   ```python
   if len(xs) <= 100:
       lost_counter += 1
       if last_valid_center and lost_counter < 10:
           center_x, center_y = last_valid_center
       else:
           center_x = center_y = None
   ```
   - 라인이 일시적으로 감지되지 않더라도, 최근 탐지된 좌표를 일정 프레임 동안 유지시켜 RC카가 부드럽게 주행하도록 함

4. **🔌 아두이노로 좌표 전송**
   ```python
   if center_x is not None:
       arduino.write(f\"{center_x},{center_y}\\n\".encode())
       response = arduino.readline().decode().strip()
   ```
   - 감지한 중심 좌표를 아두이노로 시리얼 통신을 통해 전송
   - RC카의 조향 및 ESC 제어는 이 값을 기반으로 처리됨

5. **📡 WebRTC를 통한 영상 스트리밍**
   ```python
   video_frame = VideoFrame.from_ndarray(color_frame, format=\"bgr24\")
   video_frame.pts = pts
   return video_frame
   ```
   - 처리된 프레임을 WebRTC용 비디오 프레임 객체로 변환한 뒤, 브라우저로 전송
   - 브라우저에서는 실시간으로 라인 검출 상태 확인 가능
---

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

## 🖼️ 시각 자료 예시, 📺 Youtube 시연 영상 (사진을 클릭하면 영상 재생)

- **라인 인식 예시**

<table>
  <tr>
    <td style="text-align: center;">
      <a href="https://youtube.com/shorts/51LlgFAMZnk" target="_blank">
        <img src="https://img.youtube.com/vi/51LlgFAMZnk/hqdefault.jpg" width="300px">
      </a>
      <div>🚗 일직선 자율주행</div>
    </td>
    <td style="text-align: center;">
      <a href="https://youtube.com/shorts/J_yHN7ol2DM" target="_blank">
        <img src="https://img.youtube.com/vi/J_yHN7ol2DM/hqdefault.jpg" width="300px">
      </a>
      <div>🧪 시범 자율주행</div>
    </td>
    <td style="text-align: center;">
      <a href="https://youtube.com/shorts/NI0ZMiQrU_s" target="_blank">
        <img src="https://img.youtube.com/vi/NI0ZMiQrU_s/hqdefault.jpg" width="300px">
      </a>
      <div>✅ 실제 구현 완료된 자율주행</div>
    </td>
  </tr>
</table>


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
## 🚗 WebRTC 기반 라인트레이싱 자율주행 RC카 프로젝트 느낀점

이번 **WebRTC 기반 라인트레이싱 자율주행 RC카 프로젝트**를 진행하면서 다양한 기술적 도전과 성장을 경험할 수 있었습니다.  
특히 **하드웨어와 소프트웨어가 유기적으로 결합된 시스템을 설계하고 구현**하는 과정에서 많은 것을 배웠습니다.


### 🧑‍💻 윤창혁

저는 **시스템 전체 설계 및 통합**, 그리고 **PiCamera2 영상처리 알고리즘 구현**에 집중했습니다.  
특히 **라인트레이싱 시 오프셋 기반 조향 알고리즘을 최적화**하는 과정에서 많은 시행착오를 겪었지만,  
끊임없는 테스트와 디버깅을 통해 **RC카가 안정적으로 라인을 따라 주행**하는 모습을 보며 큰 보람을 느꼈습니다.  

실제로 **하루 종일 코드를 수정하면서**, 프로그래머에게 있어 **개발뿐만 아니라 테스트와 디버깅 과정이 얼마나 중요한지**를 깊이 깨달았습니다.  
이 과정은 단순한 기능 구현을 넘어서, **하드웨어와 소프트웨어 간의 섬세한 인터페이스 설계**가 프로젝트의 완성도에 큰 영향을 미친다는 것을 알게 해주었습니다.  

또한 이번 프로젝트를 통해, 기술적인 구현만큼이나 **README 문서 작성과 기록 정리의 중요성**도 새롭게 실감할 수 있었습니다.  
**명확한 문서화는 유지보수나 협업에 필수적이며**, 프로젝트를 더욱 가치 있게 만드는 핵심 요소임을 다시금 깨닫는 계기가 되었습니다.


### 🧑‍💻 심창완

**WebRTC 서버와 웹 인터페이스 구현**을 담당하며,  
**실시간 영상 스트리밍의 복잡성**과 **웹소켓 통신의 안정성 확보**에 대해 깊이 있게 이해할 수 있었습니다.

특히 `aiortc` 기반의 WebRTC 서버를 최적화하여 **지연 시간을 최소화**하는 과정은 도전적이었지만,  
**사용자에게 끊김 없는 실시간 영상**을 제공할 수 있게 되어 뿌듯합니다.

또한 **라인 이탈 시 복원/대응 알고리즘**을 개발하면서  
**예측 불가능한 상황에 대한 시스템의 견고성**을 높이는 데 기여할 수 있었습니다.


### 🤝 팀워크와 배움

이번 프로젝트를 통해 **팀원들과의 긴밀한 협업의 중요성**을 다시 한번 느꼈습니다.  
각자의 역할에 최선을 다하면서도 서로의 어려움을 함께 고민하고 해결해 나가는 과정이 있었기에  
**성공적으로 프로젝트를 마무리**할 수 있었습니다.

앞으로도 이러한 경험을 바탕으로 **더욱 복잡하고 흥미로운 프로젝트에 도전**하고 싶습니다.

---

## 📜 라이선스
[MIT License](LICENSE) — LICENSE 파일 참조
