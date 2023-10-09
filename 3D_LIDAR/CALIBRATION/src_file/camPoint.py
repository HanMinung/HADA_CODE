from variable import *

def mouse_callback(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:
        print("pixel Coorindate : ({}, {})".format(x, y))

def main():

    cap = cv.VideoCapture(cv.CAP_DSHOW)

    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    cv.namedWindow("Camera")
    cv.setMouseCallback("Camera", mouse_callback)

    while True:

        ret, frame = cap.read()
        
        if not ret:
            break

        cv.imshow("Camera", frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # 해제
    cap.release()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
