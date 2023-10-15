import cv2

VIDEO = "C:/openCV_image/cat.mp4"

# Video 출력 class
capture = cv2.VideoCapture(VIDEO)

while cv2.waitKey(33) < 0:
    
    # capture.get : video 속성 반환 method ( sequence : present frame 수 & 총 frame 수 )
    if capture.get(cv2.CAP_PROP_POS_FRAMES) == capture.get(cv2.CAP_PROP_FRAME_COUNT):
        
        capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
        # initialize video's current frame
        
    ret, frame = capture.read()
    cv2.imshow("VideoFrame", frame)
    
    
capture.release()
cv2.destroyAllWindows()