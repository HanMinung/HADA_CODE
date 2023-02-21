# Camera & 3D LIDAR Calibration

Date : 2023.02.16 (Start)

Writer : Minung Han

Purpose : Perceived object distance estimation through lidar and camera calibration

------



## 1. Process

* 센서 퓨전을 위해서는, 먼저 좌표계를 통일시켜야 한다.

* 좌표계의 종류

  <img src="https://user-images.githubusercontent.com/99113269/219281762-0a1b446e-695a-4713-8926-5d1aec2eecfb.png" alt="image" style="zoom:50%;" />

* 라이다에서 우리가 보는 데이터는 라이다를 중심으로 하는 world frame이다.

* 그렇기에, world frame을 camera frame으로 변환해주어야 한다.

* 변환식은 다음과 같다.

  <img src="https://user-images.githubusercontent.com/99113269/219293983-242444b0-4a41-493d-a3f6-6a63967d13d9.png" alt="image" style="zoom:45%;" />

* R matrix : 3 by 3 회전 변환 행렬

  * T matrix : camera와 라이다의 위치 차이

* 위 과정을 거치게 되면, 라이다에서 본 데이터들을 카메라 중심점으로 놓고 볼 수있게 된다. 

* 주의할 점

  * 라이다 : 실제 m단위를 사용하는 반면, 카메라의 이미지는 pixel이 기본 단위이다.
  * 원근법을 반영하기 위해, camera frame의 데이터를 pixel frame으로 옮겨야 한다.
    * pixel frame : 카메라 초점으로부터 거리가 1인 가상 평면
  * Camera frame의 X Y 좌표를 Z값으로 나눠주면서 원근법 문제를 해결

* 초점거리를 나타내는 f : focal length

* skew : 비대칭계수로, 0으로 설정해도 무방하다.



## 2. Intrinsic parameter estimation

* checker board generator : https://markhedleyjones.com/projects/calibration-checkerboard-collection

* tool : cameraCalibrator in MATLAB

* size : 45mm 5x3 (A4) checker board was used

* Process 

  <img src="https://user-images.githubusercontent.com/99113269/219533110-3b068c79-b419-49b2-a1b6-7d0ca016afa4.png" alt="image" style="zoom:30%;" /><img src="https://user-images.githubusercontent.com/99113269/219533367-4b1a2f4d-350c-4921-821c-cfde908f25f6.png" alt="image" style="zoom:30%;" />

* 그 후에, calibration을 진행하게 되면, 카메라 내부 파라미터가 나오게 되고, mat file로 저장할 수 있는 형태가 된다.



## 3. Extrinsic parameter estimation

* How to calculate [R|t] extrinsic matrix ?

  * 찾아본 방법에는 두 가지가 존재한다.

    * 네모난 표지판을 공중에 띄어 라이다 실제 데이터와 카메라 이미지상에서 코너를 검출하여 코너간의 관계를 비교

      참고 : https://gnaseel.tistory.com/39?category=907758

    * 원의 코너들을 검출하여 라이다 데이터와 카메라 이미지상에서 원의 지름을 가지고 비교

      참고 논문 : https://www.mdpi.com/2073-8994/12/2/324

* 두 번째 방법을 참고하여 진행해볼 계획이다.

* 





## Reference

* Basic of sensor fusion

  * https://gnaseel.tistory.com/39?category=907758

  * https://gnaseel.tistory.com/44?category=907758

  * https://darkpgmr.tistory.com/32

  * https://medium.com/mlearning-ai/a-single-camera-3d-functions-fdec7ffa9a83

  * https://www.mdpi.com/1424-8220/14/3/5333

* Finding intrinsic elements of camera using MATLAB

  * https://blog.naver.com/mingu216/221255976037