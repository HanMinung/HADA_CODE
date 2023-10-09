# 3D lidar camera calibration

@ 작성자 : 한민웅

@ 2023 spring semester

@ 창작 모빌리티 경진대회 자율주행 구현 코드

@ School of Mechanical and Control Engineering, Handong Global University

-------------------

본 repository는 `2023 대학생창작모빌리티 경진대회`에 참여하는 한동대학교 HADA팀 코드 중 일부입니다.

3D 라이다와 camera의 world 좌표계를 고려한 calibration을 진행하고, YOLO V8을 활용하여 인식한 객체에 대한 정확한 거리값을 측정하는 것이 주요 목적입니다.

시스템의 FLOW는 다음과 같습니다.

* Camera code

  * Deep learning based image processing code
  * Send class number, bounding box center point of detected object to processing code with shared memory

* Lidar processing code

  * Project 3D point cloud data on pixel coordinate considering specifications of camera

  * Calibration verification

  * Selection of projected point nearest with bounding box point

  * Access to original point cloud data & Get distance information of detected object

    

<img src="https://github.com/HanMinung/Robotarm_Automation/assets/99113269/0cf9a3aa-de8c-4192-9cbd-ef60b21837f7" alt="제목 없는 다이어그램 drawio" style="zoom: 80%;" />



