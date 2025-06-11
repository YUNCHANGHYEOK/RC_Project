import asyncio  # 여러 작업을 동시에 실행하기 위해 사용하는 파이썬 비동기 입출력 모듈
import json  # 데이터를 주고받을 때 문자열(JSON 포맷)로 변환해주는 모듈
import cv2  # OpenCV - 이미지 처리 및 컴퓨터 비전 라이브러리
import serial  # 아두이노와의 시리얼(USB) 통신을 위한 모듈
import numpy as np  # 행렬, 수치 계산을 위한 과학 연산 라이브러리
import time  # 현재 시간 확인 또는 지연(sleep) 처리용 모듈
from av import VideoFrame  # aiortc에서 사용할 수 있는 비디오 프레임 형식
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack  # WebRTC 연결을 위한 클래스들
import websockets  # 웹소켓 서버를 만들기 위한 모듈
from picamera2 import Picamera2  # 라즈베리파이에서 카메라를 제어하기 위한 라이브러리

# 아두이노와 시리얼 통신을 설정합니다. 포트 이름은 실제 연결된 장치에 따라 다를 수 있습니다.
arduino = serial.Serial('/dev/ttyUSB0', 9600)  # /dev/ttyUSB0 포트에 9600bps 속도로 연결
arduino.timeout = 0.1  # 데이터를 읽을 때 최대 0.1초 대기 후 중단하도록 설정

# WebRTC를 통해 영상 데이터를 전달할 클래스 정의
class CameraStream(VideoStreamTrack):  # aiortc의 VideoStreamTrack 상속
    def __init__(self, ws=None):
        super().__init__()  # 부모 클래스 초기화
        self.picam2 = Picamera2()  # 카메라 객체 생성
        self.picam2.preview_configuration.main.size = (640, 480)  # 영상 해상도 설정
        self.picam2.preview_configuration.main.format = "RGB888"  # 영상 포맷 설정 (컬러)
        self.picam2.configure("preview")  # 프리뷰 설정 적용
        self.picam2.start()  # 카메라 시작
        self.ws = ws  # WebSocket 객체 (영상 외에 좌표도 보내기 위해 사용)
        self.prev_center = None  # 이전 프레임에서 찾은 중심점 저장
        self.last_valid_center = None  # 마지막으로 유효했던 중심점 저장
        self.lost_counter = 0  # 중심점을 찾지 못한 횟수 누적
        self.max_lost_frames = 10  # 최대 10번까지는 이전 중심점을 유지
        self.last_send_time = time.time()  # 마지막으로 데이터를 전송한 시간 기록

    async def recv(self):  # WebRTC에서 영상 프레임을 요청할 때 실행되는 함수
        try:
            # 카메라에서 현재 프레임(이미지)을 가져옵니다.
            frame = self.picam2.capture_array()
            if frame is None or frame.shape[0] == 0:
                print("❌ 프레임 수신 실패", flush=True)  # 프레임이 비었으면 재시도
                await asyncio.sleep(0.01)
                return await self.recv()

            # BGR 이미지를 HSV 색공간으로 변환 (색 기반 처리에 용이)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 어두운 부분만 추출하기 위한 범위 설정
            lower_black = np.array([0, 0, 0])  # 최소 HSV 값
            upper_black = np.array([180, 90, 170])  # 최대 HSV 값
            mask = cv2.inRange(hsv, lower_black, upper_black)  # 범위 안에 있으면 흰색(255), 아니면 검정(0)

            # 잡음을 제거하기 위한 형태학적 연산 (구멍 채우기)
            kernel = np.ones((3, 3), np.uint8)  # 3x3 크기의 커널 생성
            closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 닫힘 연산 적용

            color_frame = frame.copy()  # 영상 시각화를 위한 복사본 생성
            center_x = center_y = None  # 중심 좌표 초기화

            # ROI1: 영상 하단 부분, 차량 제어용으로 중심점 탐지
            roi1_start, roi1_end = 320, 400  # ROI1 범위 지정 (세로 방향)
            roi1_img = closed[roi1_start:roi1_end, :]  # ROI1 이미지를 잘라냄
            cv2.rectangle(color_frame, (0, roi1_start), (color_frame.shape[1], roi1_end), (0, 255, 0), 1)  # 시각화

            ys1, xs1 = np.where(roi1_img == 255)  # 흰색 픽셀 좌표만 추출
            if len(xs1) > 100:
                # 흰색 점이 충분하면 직선을 추정하여 중심 계산
                points1 = np.array(list(zip(xs1, ys1)), dtype=np.int32)
                [vx, vy, x0, y0] = cv2.fitLine(points1, cv2.DIST_L2, 0, 0.01, 0.01)  # 직선 피팅
                t = ((roi1_end - roi1_start) // 2 - y0) / vy  # 수직 중간 위치 계산
                cx = int(x0 + vx * t)  # 중심 x좌표 계산
                cy = int(y0 + vy * t) + roi1_start  # 중심 y좌표 계산 (전체 이미지 기준)
                center_x, center_y = cx, cy  # 중심 좌표 저장
                self.last_valid_center = (center_x, center_y)  # 유효한 중심으로 기록
                self.lost_counter = 0  # 탐지 실패 카운터 초기화

                # 중심점 시각화 (원 2개)
                cv2.circle(color_frame, (center_x, center_y), 2, (255, 0, 255), 1)
                cv2.circle(color_frame, (center_x, center_y), 2, (0, 255, 255), -1)
                print(f"📌 ROI1 중심: ({center_x},{center_y})", flush=True)
            else:
                # 중심점 탐지 실패 처리
                self.lost_counter += 1
                if self.last_valid_center and self.lost_counter < self.max_lost_frames:
                    # 최근 중심을 계속 사용
                    center_x, center_y = self.last_valid_center
                    print(f"🔁 이전 ROI1 좌표 유지: ({center_x},{center_y})", flush=True)
                else:
                    # 중심점 무효화
                    center_x = center_y = None
                    self.last_valid_center = None
                    print("❌ ROI1 라인 감지 실패", flush=True)

            # ROI2: 중간 상단 영역 - 예측 분석용 (조향 예측 등)
            roi2_start, roi2_end = 180, 260
            roi2_img = closed[roi2_start:roi2_end, :]
            cv2.rectangle(color_frame, (0, roi2_start), (color_frame.shape[1], roi2_end), (0, 255, 0), 1)

            ys2, xs2 = np.where(roi2_img == 255)
            if len(xs2) > 100:
                points2 = np.array(list(zip(xs2, ys2)), dtype=np.int32)
                [vx2, vy2, x02, y02] = cv2.fitLine(points2, cv2.DIST_L2, 0, 0.01, 0.01)
                t2 = ((roi2_end - roi2_start) // 2 - y02) / vy2
                cx2 = int(x02 + vx2 * t2)
                cy2 = int(y02 + vy2 * t2) + roi2_start
                cv2.circle(color_frame, (cx2, cy2), 2, (0, 0, 255), -1)  # 빨간 점으로 표시
                print(f"🔮 ROI2 예측 중심: ({cx2},{cy2})", flush=True)

            # 중심 좌표를 아두이노 또는 WebSocket으로 전송
            now = time.time()
            if center_x is not None and (center_x, center_y) != self.prev_center and (now - self.last_send_time) > 0.1:
                self.prev_center = (center_x, center_y)  # 마지막 전송 좌표 기록
                self.last_send_time = now  # 마지막 전송 시간 기록
                try:
                    # 아두이노로 x,y 좌표 전송
                    arduino.write(f"{center_x},{center_y}\n".encode())
                    print(f"📤 아두이노로 전송: {center_x},{center_y}", flush=True)
                    response = arduino.readline().decode().strip()
                    if response:
                        print(f"↩️ 아두이노 응답: {response}", flush=True)
                except serial.SerialException as e:
                    print(f"❌ 아두이노 전송 실패: {e}", flush=True)

                # WebSocket 클라이언트에도 전송
                if self.ws:
                    try:
                        await self.ws.send(json.dumps({"x": center_x, "y": center_y}))
                    except:
                        pass
            elif center_x is None:
                # 탐지 실패 시 -1 전송
                arduino.write(b"-1,-1\n")

            await asyncio.sleep(0.005)  # 프레임 간 딜레이
            pts, time_base = await self.next_timestamp()  # 프레임 타임스탬프 계산
            video_frame = VideoFrame.from_ndarray(color_frame, format="bgr24")  # 영상 프레임 생성
            video_frame.pts = pts
            video_frame.time_base = time_base
            return video_frame

        except Exception as e:
            print(f"❌ recv() 내부 오류: {e}", flush=True)
            await asyncio.sleep(0.01)
            return await self.recv()  # 오류 시 재시도

    def stop(self):
        self.picam2.stop()  # 카메라 종료

# WebRTC + WebSocket 연결을 처리하는 함수
async def handler(websocket):
    print("🔌 WebSocket client connected")
    pc = RTCPeerConnection()  # WebRTC 피어 연결 객체 생성
    stream = CameraStream(ws=websocket)  # 영상 스트림 생성
    pc.addTrack(stream)  # 피어 연결에 스트림 추가

    try:
        async for message in websocket:  # 클라이언트 메시지 수신 반복
            data = json.loads(message)
            if data.get("type") == "offer":  # SDP offer 수신 시 처리
                offer = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                await pc.setRemoteDescription(offer)  # 상대방 설명 저장
                answer = await pc.createAnswer()  # 응답 생성
                await pc.setLocalDescription(answer)  # 응답 설정
                await websocket.send(json.dumps({  # 응답을 클라이언트로 전송
                    "sdp": pc.localDescription.sdp,
                    "type": pc.localDescription.type
                }))
                print("✅ WebRTC 연결 완료")
    except Exception as e:
        print("❌ WebSocket 오류:", e)
    finally:
        await pc.close()  # WebRTC 종료
        stream.stop()  # 카메라 스트림 종료
        print("🔌 WebSocket 연결 종료")

# WebSocket 서버를 실행하는 메인 함수
async def main():
    print("🌐 WebSocket 서버 시작: ws://0.0.0.0:8765")
    async with websockets.serve(handler, "0.0.0.0", 8765):  # 8765번 포트에서 서버 실행
        await asyncio.Future()  # 서버가 종료되지 않도록 무한 대기

# 프로그램 시작 지점
if __name__ == "__main__":
    asyncio.run(main())  # 메인 함수 실행 (비동기 방식)
