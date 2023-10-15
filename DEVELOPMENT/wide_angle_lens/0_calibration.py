from module import *

CHECKERBOARD = (13, 8)

objpoints = [] 
imgpoints = [] 

objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(-1, 2)

images = glob.glob('./raw_images/*.jpg')

for fname in images :
    
    img = cv.imread(fname)
    
    if img is None:
        
        print(f"Failed to load image at {fname}")
        continue
    
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    ret, corners = cv.findChessboardCorners(gray, CHECKERBOARD, cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK+cv.CALIB_CB_NORMALIZE_IMAGE)

    if ret == True:
        
        objpoints.append(objp)
        imgpoints.append(corners)

        cv.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
        cv.imshow('img',img)
        cv.waitKey(500)
        
    else:
        
        print(f"Failed to find chessboard corners in {fname}. Skipping...")

cv.destroyAllWindows()

img_size = gray.shape[::-1]

try:

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, img_size, None, None)

    # print("Camera matrix : \n")
    # print(mtx)
    # print("dist : \n")
    # print(dist)
    # print("rvecs : \n")
    # print(rvecs)
    # print("tvecs : \n")
    # print(tvecs)
    
except cv.error as e:
    print(f"Error occurred during calibration: {e}")
