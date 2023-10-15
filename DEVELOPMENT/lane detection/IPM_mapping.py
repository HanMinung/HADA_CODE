import cv2 as cv
from module import *

undist  =  UNDISTORT()
wide    =  CALIBRATION_WIDE_LENS()
ipm     =  IPM()

def main():
    
    input_video_path   =  "video_file/test_video.mp4"

    cap = cv.VideoCapture(input_video_path)

    if not cap.isOpened() : return

    cv.namedWindow("Original Frame")

    while cap.isOpened() :
        
        ret, frame = cap.read()

        if not ret : break

        ipm_frame = ipm.ipm_transform(undist.undistort(frame))

        cv.imshow("Original Frame" , frame)
        cv.imshow("IPM Frame"      , ipm_frame)

        k = cv.waitKey(27) & 0xFF

        if k == ord('q')   : break
        
        elif k == ord('s') : cv.waitKey()

    cap.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    
    main()
