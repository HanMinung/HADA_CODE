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



## 3. Extrinsic matrix

* Process

  <img src="https://user-images.githubusercontent.com/99113269/229337868-afb5e1ea-5a8d-40fd-9e8a-9c0184886d03.png" alt="image" style="zoom: 67%;" />

* Matlab simulation

  ```matlab
  clear; close all; clc;
  
  % YOLO image sjze : 620 x 480
  
  lidarData  = pcread("../calibration/image,PCD/pointcloud.pcd");
  
  Alpha = pi/2;  Beta = 0;   Gamma = 0;
  
  camera_height = -0.2;
  camera_recede = 0.0;
  
  imgWidth  = 620;
  imgHeight = 480;
  
  camImage = zeros(imgHeight,imgWidth);
  focalLen = 0.00367;
  
  Rx = [1       0            0        ; 
        0  cos(Alpha)   -sin(Alpha)   ;
        0  sin(Alpha)   cos(Alpha)]   ;
  
  Ry = [cos(Beta)     0       sin(Beta) ;
            0         1           0     ;
        -sin(Beta)    0       cos(Beta)];
  
  Rz = [cos(Gamma)   -sin(Gamma)      0 ; 
        sin(Gamma)    cos(Gamma)      0 ;
              0           0           1];
  
  rotMat = Rz * Ry * Rx;
  
  transMat = [0 ; camera_height ; camera_recede];
  
  extMat = [rotMat transMat];
  
  figure(1);
  pcshow(lidarData);
  
  yCoords = lidarData .Location(:,2);
  negativeYIndices = find(yCoords < 0);
  lidarData = select(lidarData, setdiff(1:size(lidarData.Location,1), negativeYIndices));
  XYZ = lidarData.Location;
  
  figure(2);
  pcshow(lidarData);  title("Preprocess");
  
  FOV_x = 48;
  FOV_y = 42;
  
  M_ext = [1 0 0 0;
           0 0 -1 camera_height;
           0 1 0  camera_recede];
  
  
  ox = imgWidth/2;
  oy = imgHeight/2;
  sx = focalLen * tand(0.5 * FOV_x)/(0.5 * imgWidth) ; 
  sy = focalLen * tand(0.5 * FOV_y)/(0.5 * imgHeight) ; 
  
  
  M_int= [focalLen/sx     0                   ox;
           0              focalLen/sy         oy;
           0                   0                   1 ];
  
  x_im = zeros(length(XYZ),1);
  y_im = zeros(length(XYZ),1);
  
  for i=1:length(XYZ)
  
      Cz = XYZ(i,2) + camera_recede;
      MAT =1/Cz * M_int * M_ext * [XYZ(i,1);XYZ(i,2);XYZ(i,3); 1];
  
      x_im(i) = MAT(1);
      y_im(i) = MAT(2);
  end
  
  Projected = horzcat(x_im, y_im);
  
  % rgb = insertShape(cam_image,'circle',[(img_width-x_im) (img_height-y_im) ones(length(XYZ),1)],'LineWidth',5,'Color', 'white');
  rgb = insertShape(camImage,'circle',[(imgWidth - x_im) (y_im) ones(length(XYZ),1)],'LineWidth',5,'Color', 'white');
  figure(3)
  imshow(rgb)
  title("Projected PCD data",FontSize = 14);
  xlabel("pixel X",FontSize = 14); ylabel("pixel Y",FontSize = 14);
  ```

  

  

  

  

