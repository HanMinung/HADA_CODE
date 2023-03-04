# Velodyne Interface

Writer : 한민웅

Date : 2023.02.09

Purpose : VLP-16 3D Lidar interface construction and its application

Sensor : Velodyne VLP-16

Environment : Windows 10, Python, Anaconda 3

-------

[TOC]

## 1. Veloview

* Installation & Guide : 

  [Veloview installation]: https://www.paraview.org/veloview/

* Image environment : mschool

<img src="https://user-images.githubusercontent.com/99113269/217719587-79a72c50-d984-4370-9957-f76124ee91a7.png" alt="image" style="zoom: 40%;" />



## 2. Basic information

* reference : VLP-16 user manual

------------

### 2.1. Horizontal resolution

* RPM : 600
  * RPM은 주로 60의 배수로 설정하며, 수정할 필요없음.
* resolution : 0.2 [deg]

<img src="https://user-images.githubusercontent.com/99113269/217737397-febcbc4e-5bb2-4df7-88c1-589fa16ce145.png" alt="image" style="zoom:40%;" />



### 2.2. Vertical angles and ID

<img src="https://user-images.githubusercontent.com/99113269/217720695-6f8fb59e-b64a-4a6e-97c2-2cbcd15ef198.png" alt="image" style="zoom:50%;" />

* VLP16의 특이점 : vertical laser 16개의 순서에 따라 -15 [deg], 1 [deg], -13[deg], 3 [deg] ... 의 순서로 되어있다.



### 2.2. Single return mode data structure

<img src="https://user-images.githubusercontent.com/99113269/217721501-532803af-f947-42f4-9644-f5dd5d015f13.png" alt="image" style="zoom:40%;" />

<img src="https://user-images.githubusercontent.com/99113269/217722149-7cd21e0a-c977-40a2-bc02-a2abe18d2fe9.png" alt="image" style="zoom: 50%;" />

* 하나의 데이터 packet : 1248 byte
* 12개의 데이터 block이 존재
* 12 * ( 2 bytes **flag** + 2 bytes **azimuth** + 32 * ( 2 bytes **distance** + 1 byte **reflectivity** ) ) = 1200 bytes





## 3. Code

```python
HOST = "192.168.1.201"
PORT = 2368

LASER_ANGLES = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
NUM_LASERS = 16
```

* 고유 IP, Port 번호 지정 (고정)
* vertical laser 인덱스 순서에 맞는 각도값을 위와 같은 형태로 저장



### 3.1. Convert python 2 to 3

* 기존의 코드가 python 2의 버전으로 되어 있기 때문에, python 3 버전으로 바꾸어야 한다.

* 먼저, velodyne.py 파일이 있는 폴더 내에 2to3.py 파일을 넣어준다.

* python 파일이 있는 폴더에 들어가 경로에 `cmd`를 입력한다.

* 그 후, cmd 창에 다음 명령어를 입력

  ```python
  python 2to3 -w filename.py
  
  실제 실행한 명령어 : python 2to3 -w velodyne.py
  ```

* 다음 과정을 거치면, 원본 파일은 filename.py.bak file의 형태로 (백업), 수정된 파일은 filename.py의 형태로 저장이 된다.

  <img src="https://user-images.githubusercontent.com/99113269/217735971-6f0b25a4-0c77-4b88-90ce-4cb016f4e0a0.png" alt="image" style="zoom:60%;" />



### 3.2. 기존 코드

Original code :

* 기존 코드를 다음과 같이 경로 생성을 한 뒤에 아나콘다 가상환경에서 돌리면, bin file이 생성됨을 알 수 있다.

* anaconda에서 다음 명령어들을 실행하여 코드 실행이 가능하다.

* python file을 실행할 때, read와 경로가 의미하는 바는 다음과 같다.

  * read : 인자 중 첫 번째
  * dir : bin file이 저장될 경로

  ```python
  - conda activate py39 [본인의 가상환경]
  
  - cd [velodyne.py 파일이 있는 경로]
  	* D drive라면 cd 대신 D:
      
  - python velodyne.py read [bin file이 저장될 경로]
  ```

  <img src="https://user-images.githubusercontent.com/99113269/217752135-44af7f86-9486-4707-a598-cd1e2abc3dcc.png" alt="image" style="zoom: 67%;" />

  

* 그 후, 다음 과정을 통해 bin file을 unpack할 수 있다.

  * unpack 과정을 거치게 되면, 해당하는 x,y,z 좌표가 나오게 되는데, 이에 대한 데이터 해석을 위해, bin 파일을 어디서 적절하게 끊어가면서 만들지 고민을 해야한다.

* anaconda에서 다음 명령어들을 실행하면 unpack 과정이 이루어진다

  ```python
  - conda activate py39 [본인의 가상환경]
  
  - cd [velodyne.py 파일이 있는 경로]
  	* D drive라면 cd 대신 D:
      
  - python velodyne.py unpack [bin file이 있는 경로]
  ```

  ![image](https://user-images.githubusercontent.com/99113269/217754144-50871c1f-b867-46e5-afd0-4cd1ba5f0307.png)

  



### 3.3. 코드 수정 계획

* 600 RPM으로 회전하기 때문에, 0.1초에 한 바퀴를 돌게 된다.
* 스캔한 데이터를 계속 누적을 하되, 0.1초마다 새로운 bin file을 생성할 수 있도록 만들 예정이다. 360 [deg] 스캔한 데이터에 대한 분석을 하기 위해.
* 최종적으로는, 관심 영역 (-90 [ deg ] ~ 90 [ deg ])에 대한 데이터만 받으면서, 데이터의 사이즈를 줄이고, 필요한 가용 정보들을 뽑아내고자 한다.
  * 정면각도를 설정하고자 한다면, 고유 IP 사이트에서 FOV start - FOV end를 설정하면 된다.
  * http://192.168.1.201/
  * FOV start : 270 
  * FOV end : 90

<img src="https://user-images.githubusercontent.com/99113269/218389412-5cdad9ed-8ac7-4cd7-9366-3a63e4ae63e7.png" alt="image" style="zoom: 40%;" />



### 3.4. 코드 기본 개념



#### 3.4.1. argv, multiprocessing

* CPU 분할 작업을 통한 처리 속도 향상

* sample code and result

  ```python
  import sys
  from multiprocessing import Process
  
  def adder(fac1, fac2) : 
      sum = fac1 + fac2
      print(sum)
  
  def sub(fac1, fac2) :
      sub = fac1 - fac2
      print(sub)
  
  
  if __name__ == '__main__' :
      
      if(sys.argv[1] == 'calculation') :
          
          processA = Process(target = adder, args = (5, 3))
          processA.start()
          
          processB = Process(target = sub, args = (5, 3))
          processB.start()
  ```

  <img src="https://user-images.githubusercontent.com/99113269/218397616-14cbacc5-361a-4b7d-ad2d-85b86e40d52d.png" alt="image" style="zoom: 67%;" />





#### 3.4.2. saving package

* 파일 생성 함수

  * open : 인자로 파일 경로, mode를 넣어준다.
  * code에서 구현한 'ab'의 의미 :
    * a : 이미 파일이 존재하면 그 뒤에 이어쓰는 append 형식의 mode
    * b : binary mode

* os module에서 사용한 함수

  * os.path.join
    * 주로 경로와 파일명을 결합하거나, 분할된 경로를 하나로 합칠 때 사용

* sample code and result

  ```python
  import os
  import time
  from datetime import datetime
  
  dir = 'C:/Users/hanmu/Desktop/Camera/pythonprac/test'
  
  for idx in range(3) :
      
      file_fmt = os.path.join(dir, '%Y-%m-%d_%H%M')
      path = str(datetime.now().strftime(file_fmt)) + '_' + str(idx) +'.txt'
  
      print('save to', path)
      fp = open(path, 'ab')
  ```

  <img src="https://user-images.githubusercontent.com/99113269/218402304-973d507b-8fb7-4ca2-a3f4-d75c58e44674.png" alt="image" style="zoom:67%;" />



#### 3.4.3. saving CSV

* CSV file 생성

  * 함수 : open( 'file path/name', 'mode' , option )
  * 사용한 옵션 : newline = ' ' 
    * 빈 공백 없이 한줄씩 쓰도록

* code and result

  ```python
  import csv
  
  w = [[1,2,3], [4,5,6], [4,1,3], [4,3,5], [1,1,1], [4,2,6]]
  
  
  with open('./test/test.csv','w', newline = '') as f : 
      
      wt = csv.writer(f)
      
      for i in w : 
          wt.writerow(i)
  ```

  <img src="https://user-images.githubusercontent.com/99113269/218670820-2433a423-1c5c-439a-b4f0-9501b52b286d.png" alt="image" style="zoom: 33%;" />





### 3.5. CSV data plot

* MATLAB으로 plot을 하기위해 설치한 addon box

  * computer vision toolbox

* MATLAB code

  ```matlab
  close all; clear; clc;
  
  addpath('path');
  
  rawdata = readmatrix('filename.csv');
  x = rawdata(:,1);                
  y = rawdata(:,2);
  z = rawdata(:,3);
  
  pcshow([x y z])
  ```

* unpack을 통해 저장된 CSV file을 plot 했을때 나오는 결과는 다음과 같다.

* 좌 : FOV start - end = 0 ~ 359 

  우 : FOV start - end = 270 ~ 90

  <img src="https://user-images.githubusercontent.com/99113269/218952315-5b0ef6d3-cf2e-4114-bf22-657417588be9.png" alt="image" style="zoom:50%;" />





### 3.6. Real time code 구현

* 먼저, real time code를 구현하기 위해서는, save package 과정과 unpack 과정이 multiprocess의 형태로 타이밍이 맞게 이루어져야 한다.

* 구조에 따르면, 한 프로세스에서는 bin file을 저장하고, 다른 프로세스에서는 그러한 bin file을 해석하여 XYZ 결과를 저장한다.

* 이 구조가 완성이 되면, 다른 프로세스 하나를 열어, 실시간 구조의 형태로 IPC 통신을 통해 다른 프로세스에 raw 데이터를 넘겨 처리하면 될것이다.

* 추가한 실시간 구현의 코드 구조는 다음과 같고, real time 구조 안에서 데이터 처리 or IPC 통신을 통해 다른 프로세스에 데이터를 넘겨주는 작업을 하면 될 것이다.

  ```python
  if __name__ == "__main__" :
      
      decode = DECODE()
      
      print("\n\n------------------------------------------------------------------")
      print("                      PROGRAM START...!                             \n")
      print("             - SAMPLING TIME  :  {}".format(time_ts))                       
      print("             - FINAL TIME     :  {}".format(time_final))
      print("----------------------------------------------------------------------")
      
      top_dir = 'binfile_' + datetime.now().strftime('%H_%M_%S')
      
      processA = Process(target = capture, args = (PORT, DATA_QUEUE))
      processA.start() 
      processB = Process(target = save_package, args = (sys.argv[1] + '/' + top_dir, DATA_QUEUE))
      processB.start()
      
      time_start = time.time()
      
      while (time_stime < time_final) :
  
          decode.unpack(os.path.join(sys.argv[1], top_dir))
          
          while(1) :
          
              time_curr = time.time()
              time_del = time_curr - time_start - time_stime
              
              if (time_del > time_ts):
                  
                  time_cnt += 1
                  time_stime = time_cnt * time_ts
                  
                  break
      
      processA.terminate()
      processB.terminate()
      
      print("PROCESS A and B IS TERMINATED ...!")
      print("PROGRAM FINISHED ...!")
  ```
  
