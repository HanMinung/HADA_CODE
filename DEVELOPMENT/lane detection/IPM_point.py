from module import *

wide = CALIBRATION_WIDE_LENS()
ipm  = IPM()


clicked_x = -1
clicked_y = -1

def on_mouse_click(event, x, y, flags, param):

    global clicked_x, clicked_y
    
    if event == cv.EVENT_LBUTTONDOWN:
        clicked_x, clicked_y = x, y

def main():
    # Input video file path
    video_path = "video_file/test_video.mp4"

    # Open the video file
    cap = cv.VideoCapture(video_path)

    # Create a window to display the video frame
    cv.namedWindow("Video Frame")
    cv.setMouseCallback("Video Frame", on_mouse_click)

    while cap.isOpened() :
        
        ret, frame = cap.read()

        if not ret : break

        h, w = frame.shape[:2]
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(wide.wide_camera_mat, wide.dist_coeffs, (w, h), 1, (w, h))
        
        undistorted_frame = cv.undistort(frame, wide.wide_camera_mat, wide.dist_coeffs, None, new_camera_matrix)

        ipm_frame = ipm.ipm_transform(undistorted_frame)

        cv.imshow("Video Frame", ipm_frame)

        k = cv.waitKey(27) & 0xFF
        
        if k == ord('q') : break
        
        elif k == ord('s') : cv.waitKey()
    
        global clicked_x, clicked_y
        
        if clicked_x != -1 and clicked_y != -1:
            
            print(f"Clicked Pixel Coordinates: ({clicked_x}, {clicked_y})")
            
            clicked_x, clicked_y = -1, -1

    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
