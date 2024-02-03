# AI-IoT-CCTV-Security-Service
올씨구나 - 인공지능 IoT 스마트 방범 서비스

## 프로젝트 소개
[![Video Label](http://img.youtube.com/vi/q88HHYJPSqw/0.jpg)](https://youtu.be/q88HHYJPSqw) <br> 클릭 시 유튜브로 이동합니다. <br>

CCTV + 자율주행 RC카를 이용하여 사각지대를 순찰하며 거리의 이상 현상 및 행인의 이상 행동을 감지하는 방범 서비스입니다. 수집한 정보는 실시간으로 웹사이트와 어플리케이션에 송출하고, 스마트폰으로 RC카를 원격 무선 조종할 수 있습니다.

## 개발 기간
- 2023.04.17. ~ 2023.11.15.

### 업무분장
- 멘토 유상오: 프로젝트 관리 및 지도
- 팀장 우진우: 영상처리 알고리즘 개발, 애플리케이션 개발
- 팀원 곽연규: 라즈베리파이 활용 및 sw개발, 애플리케이션 개발
- 팀원 김예진: 라즈베리파이 센서 및 서버, 데이터베이스, 웹사이트 및 애플리케이션 개발
- 팀원 정동규: RC카 자율주행, 하드웨어 개발, 애플리케이션 개발

### 개발환경
![화면 캡처 2023-12-16 003936](https://github.com/yejeeni/AI-IoT-CCTV-Security-Service/assets/110469361/81853680-fa4b-420f-b7c7-92d36462341a)

## 주요 기술
 **가) S/W**<br>

 **1) Bluetooth를 이용한 무선 통신**<br>
App과 Arduino의 무선연결을 Bluetooth를 이용한다. Bluetooth로 App과 연결되어 있는 Arduino는 App에서 실시간으로 받은 센서값의 상황을 받을 수 있다. Event가 발생했을 시 RC카의 구동을 조작하기 위해 특정 신호를 보낼 수 있도록 한다.

 **2) Web Design**<br>
![화면 캡처 2023-12-16 000750](https://github.com/yejeeni/AI-IoT-CCTV-Security-Service/assets/110469361/f3bac4b4-46e8-4214-8f3f-283ca0354bd1)
<br>
HTML, CSS, JS를 통해 제작하였다. RC카의 실시간 스트리밍 영상, 센서 수치, 현위치를 확인할 수 있다. 카카오지도 API를 사용하였다.

 **3) Web Framework**<br>
Flask를 이용하였다. 또한 파이썬을 통해 Raspberry Pi의 gpio를 제어하며 웹과 연동하여 Raspberry Pi에서 웹서버를 구동할 수 있게 하였다. 온습도/가스/GPS 센서를 제어하고 값을 데이터베이스에 저장하거나 조회한다.

 **4) Database**<br>
Google의 Firebas를 사용한다. 다양한 센서의 값을 실시간으로 저장하고 조회하며, 위험을 감지하는 데에 사용된다.

 **5) 사람 동작 인식 (OpenPose)**<br>
 ![화면 캡처 2023-12-16 000347](https://github.com/yejeeni/AI-IoT-CCTV-Security-Service/assets/110469361/04150ef0-d9b9-4d33-8cfe-b79016e1331d)
<br>
YOLOv3를 통해 인식된 사람을 bounding box로 표시해 해당 boundary에 있는 사람을 구별하기 위해 OpenPose를 이용한다. OpenPose의 body_25를 이용해 사람의 관절 포인트를 25개로 나누었고 동시에 여러 사람이 식별되도록 구현하였다. 사람의 키포인트를 추출하기 위해 Gaussian Blur와 이진화를 사용해서 더 정밀하게 관절을 찾는다. 실시간 스트리밍 영상을 일정 시간마다 캡쳐하여 캡쳐된 사진 속에 사람이 식별되면 Neck, Lwrist, Rwrist, Midhip, Lankle, Rankle으로 총 6개의 관절의 위치를 따로 저장하여 해당 위치들이 일직선상에 위치해 있다면 쓰러져 있는 사람으로 판단한다.

 **6) 애플리케이션**<br>
 ![화면 캡처 2023-12-16 000839](https://github.com/yejeeni/AI-IoT-CCTV-Security-Service/assets/110469361/701bc5ae-7b87-4bce-a34c-eaf5034dfd74)
 <br>
 MIT App Inventor를 이용하였으며, 데이터베이스에 저장된 실시간 데이터들을 받아서 화면에 보여준다. WebViewer component를 이용해 ‘통합관제 Web’과 연동하여 실시간 스트리밍이 가능하고 자율주행 RC카의 Arduino와 Bluetooth로 연동되어 자율주행 RC카를 상황에 따라 수동제어한다. 센서값이 설정한 임계치를 넘으면 알림창을 보여주고 수동조작으로 넘어갈 수 있도록 하였다.

 **나) H/W**<br>
 ![화면 캡처 2023-12-16 001048](https://github.com/yejeeni/AI-IoT-CCTV-Security-Service/assets/110469361/ee219790-71ba-4ae5-80e5-576307d5b8d9)
 <br>
 **1) 초음파 센서를 이용한 거리 인식**<br>
RC카가 이동형 CCTV의 역할을 해낼 수 있도록 전방 장애물 거리 측정을 통해 장애물을 회피하며 자율주행하도록 했다.<br><br>
 **2) 온습도, 가스, GPS 센서를 이용한 위험 감지**<br>
 현장의 온습도와 가스 수치를 통해 화재나 가스 누출 등을 탐지하고 위험이 발생한 위치를 GPS로 확인 가능하다.<br><br>
  **3) 카메라를 이용한 CCTV 스트리밍**<br>
  라즈베리파이 카메라를 통해 주행 중인 RC카가 촬영하는 영상을 송출하고, 송출 중인 영상에 접속할 수 있도록 하였다.<br><br>

## 달성 성과
-  **논문게재**: 방범 설비의 스마트화를 위한 인공지능 자율주행 CCTV 시스템, ACK 2023 학술발표대회 논문집 (30권 2호)
-  **앱 등록**: all see구나 RC카, Google Play
-  **프로그램 등록**: 올씨구나, 한국저작권등록위원회
-  **공모전**: 2023 한이음 ICT멘토링 공모전, 입선
----
 본 프로젝트는 과학기술정보통신부 정보통신창의인재양성사업의 지원을 통해 수행한 ICT멘토링 프로젝트 결과물입니다.
