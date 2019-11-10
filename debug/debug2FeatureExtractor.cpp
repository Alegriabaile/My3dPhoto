//
// Created by ale on 19-11-8.
//

//
// Created by ale on 19-11-8.
//

//
// Created by ale on 19-11-8.
//

#include "1FrameReader.h"
#include "2FeatureExtractor.h"

int main(int argc, char** argv)
{
    using namespace cv;
    using namespace std;
    using namespace m3d;

    string argFileName("argv.txt");
    if(argc>2)
        argFileName.assign(argv[1]);

    vector<Frame> frames;
    FrameReader(argFileName, frames);


    m3d::FeatureExtractor::extractFeatures(frames);

    for(unsigned int i=0; i<frames.size(); ++i)
    {
        cv::Mat image;
        cv::drawKeypoints(frames[i].image, frames[i].keypoints, image, Scalar::all(-1), 4);

        if(image.cols > 1500 || image.rows > 1000)
        {
            float rat = float(image.cols)/float(image.rows);
            cv::resize(image, image, cv::Size( rat*1000, 1000));
        }

        cv::imshow("image", image);

        cv::waitKey();
    }


    return 0;
}

