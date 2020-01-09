//
// Created by ale on 19-12-19.
//

#include <opencv2/opencv.hpp>
using namespace cv;


//crop the full screen video(W*H = 1920*1080, 30fps) to video of size W*H = 800*600
void cropTheFullScreenVideo()
{
    std::string videoName("debug11/");

    for(size_t i = 5; i<=5; ++i)
    {
        std::string index(std::to_string(i));
        VideoCapture capture(videoName + index + ".mp4");
        cv::Size sWH = cv::Size(/*Width*/ 800, /*Height*/ 600);
        VideoWriter writer(videoName + index + "_cropped.mp4", CV_FOURCC('M','P','4','V'), 30.0f, sWH);

        Mat frame;
        Mat frame_cropped;
        size_t wLength = 153, hLength = 162;
        while(capture.read(frame))
        {
            frame_cropped = frame(Range(hLength, hLength + 600), Range(wLength, wLength + 800)).clone();
            writer<<frame_cropped;
//            imshow("frame",frame);
//            imshow("frame_cropped", frame_cropped);
//            waitKey(30);
        }
    }
}

int main()
{

    cropTheFullScreenVideo();

    return 0;

}