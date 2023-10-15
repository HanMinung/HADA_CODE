#include <iostream>
#include "include/sensor/camera/camera.hpp"


int main() {

    cv::VideoCapture cap(0, cv::CAP_DSHOW);
    cap.set(cv::CAP_PROP_FPS, 60.0);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);

    if (!cap.isOpened()) {
        printf("Can't open the video");
        return -1;
    }

    cv::Mat frame;

    while (true) {

        cap >> frame;

        if (frame.empty()) {
            std::cout << "Camera is disconnect..." << std::endl;
            return 0;
        }
        cv::imshow("test", frame);
        if (cv::waitKey(1) == 27)
            break;
    }


    // a = bev

    // cv::imshow
    return 0;
}