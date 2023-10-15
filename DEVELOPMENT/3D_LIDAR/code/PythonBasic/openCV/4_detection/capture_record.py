import datetime
import cv2

video = "C:/openCV_image/cat.mp4"
store = "C:/openCV_image/capture/"

capture = cv2.VideoCapture(video)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
record = False

while True:
    if(capture.get(cv2.CAP_PROP_POS_FRAMES) == capture.get(cv2.CAP_PROP_FRAME_COUNT)):
        capture.open(video)

    ret, frame = capture.read()
    cv2.imshow("VideoFrame", frame)

    now = datetime.datetime.now().strftime("%d_%H-%M-%S")
    key = cv2.waitKey(33)

    if key == ord('q'):
        print("Terminate Video..!")
        break
    
    elif key == ord('s'):
        print("Captured..!")
        cv2.imwrite(store + str(now) + ".png", frame)
        
    elif key == 24 :
        print("Start Recording..!")
        record = True
        video = cv2.VideoWriter(store + str(now) + ".avi", fourcc, 20.0, (frame.shape[1], frame.shape[0]))
        
    elif key == 3 :
        print("Stop recording..!")
        record = False
        video.release()
        
    if record == True :
        print("Recording..!")
        video.write(frame)

capture.release()
cv2.destroyAllWindows()