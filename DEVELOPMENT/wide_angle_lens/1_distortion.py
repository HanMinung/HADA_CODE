from module import *

# imgWidth  = 640
# imgHeight = 480

# cx, cy = imgWidth/2, imgHeight/2

fx = 1.11288472e+03
fy = 1.10502895e+03

# 이미지 사이즈 절반
cx = 1.94098724e+03     
cy = 1.07627793e+03

k1 = -0.23143364
k2 = 0.05816916
p1 = 0.00346633
p2 = -0.00129636
k3 = -0.00739589

mode = 2

camera_matrix = np.array([[fx , 0  , cx],
                          [0  , fy , cy],
                          [0  , 0  , 1 ]], dtype='double')

dist_coeffs = np.array([k1, k2, p1, p2, k3])  

if mode == 1 :

    cap = cv.VideoCapture(cv.CAP_DSHOW)  

    while True:
        
        # S = time.time()
        
        ret, frame = cap.read()
        
        if not ret:
            break

        h, w = frame.shape[:2]
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
        
        undistorted_frame = cv.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)

        # T = time.time()
        
        # print(f"{T-S} [sec]")

        cv.imshow('undistorted', undistorted_frame)
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()


if mode == 2 :

    img = cv.imread('./raw_images/WIN_20230710_13_20_59_Pro.jpg')
    
    if img is None :
        print("Failed to load image")
        
    else:
        
        h, w = img.shape[:2]
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))

        undistorted_img = cv.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)
        
        undistorted_img = cv.resize(undistorted_img, dsize = (1280, 720))
        
        cv.imshow('Undistorted Image', undistorted_img)
        cv.waitKey(0)
        cv.destroyAllWindows()


