//
// Created by ale on 19-11-8.
//

#include "1FrameReader.h"

using namespace m3d;

void FrameReader::readArgv(std::string argv1)
{
    std::ifstream inFile( argv1);
    if (!inFile)
    {
        LOG("FrameReader::readArgv", "argv file does not exist.");
        exit(-1);
    }
    inFile>>dataDir>>state;
    inFile.close();
}

void FrameReader::readDefaultIntrinsic(const std::string &paramDefault, m3d::IntrinsicD& intrinsicD)
{
    double fx, fy, cx, cy;

    std::ifstream inFile(paramDefault);
    if (!inFile)
    {
        LOG("FrameReader::readDefaultIntrinsic", "paramDefault file does not exist.");
        exit(-1);
    }
    inFile>> fx>> fy>> cx>> cy;
    inFile.close();

    intrinsicD.setIntrinsic(fx, fy, cx, cy);
}

void FrameReader::readImage(const std::string &imageFileName, cv::Mat &image)
{
    image = cv::imread(imageFileName, CV_LOAD_IMAGE_ANYCOLOR|CV_LOAD_IMAGE_ANYDEPTH);
    if(image.empty())
    {
        LOG("FrameReader::readImage", "read image failed");
        exit(-1);
    }
}

void FrameReader::readImages(const std::vector<std::string> &imageFileNames, std::vector<m3d::Frame> &frames)
{
    frames.clear();
    frames.resize(imageFileNames.size());

    for(int i=0; i<imageFileNames.size(); ++i)
    {
        readImage(imageFileNames[i], frames[i].image);
        frames[i].imageFileName.assign(imageFileNames[i]);
    }
}
void FrameReader::readDepth(const std::string &depthFileName, m3d::Frame &frame)
{
    cv::Mat dTmp = cv::imread(depthFileName, CV_LOAD_IMAGE_ANYCOLOR|CV_LOAD_IMAGE_ANYDEPTH);
    if(dTmp.empty())
    {
        LOG("FrameReader::readDepth", "read depth failed");
        exit(-1);
    }
    unsigned int channels_ = dTmp.channels();
    if(channels_ != 3 && channels_ != 1)
    {
        LOG("FrameReader::readDepth", "depth.channels() != (1 or 3)");
        exit(-1);
    }

    frame.depth = cv::Mat(dTmp.rows, dTmp.cols, CV_32FC1, cv::Scalar(0));
    frame.disparity = cv::Mat(dTmp.rows, dTmp.cols, CV_32FC1, cv::Scalar(0));

    if(state == 0)
    {
        for(unsigned int h=0; h<dTmp.rows; h++)
        {
            for(unsigned int w=0; w<dTmp.cols; w++)
            {
                unsigned int d = dTmp.ptr<uchar>(h)[w*channels_];
                frame.disparity.at<float>(h,w) = float(d);
            }
        }
        frame.depth = 1/frame.disparity;
    }else if(state == 1)
    {
        for(unsigned int h=0; h<dTmp.rows; h++)
        {
            for(unsigned int w=0; w<dTmp.cols; w++)
            {
                unsigned int d = dTmp.ptr<uchar>(h)[w*channels_];
                frame.depth.at<float>(h,w) = float(d);
            }
        }
        frame.disparity = 1/frame.depth;
    }
}

void FrameReader::readDepths(const std::vector<std::string> &depthFileNames, std::vector<m3d::Frame> &frames)
{
    for(int i=0; i<depthFileNames.size(); ++i)
    {
        readDepth(depthFileNames[i], frames[i]);
        frames[i].depthFileName.assign(depthFileNames[i]);
    }
}

void FrameReader::readParameter(const std::string &paramFileName, m3d::Frame &frame)
{
    frame.paramFileName.assign(paramFileName);
    //read standalone intrinsics and the given extrinsics.
    //todo
//    m3d::IntrinsicD intrinsicD;
//    m3d::ExtrinsicD extrinsicD;
}

void FrameReader::readParameters(const std::vector<std::string> &paramFileNames, m3d::IntrinsicD &defaultIntrinsicD, std::vector<m3d::Frame> &frames)
{
    for(unsigned int i=0; i<frames.size(); ++i)
    {
        if(i == 0)
        {
            double colorh = frames[i].image.rows;
            double colorw = frames[i].image.cols;
            double depthh = frames[i].depth.rows;
            double depthw = frames[i].depth.cols;

            //0, 1, 2, 3: fx, fy, cx, cy
            double *intri = defaultIntrinsicD.intrinsic_c();

            double c[4], d[4];

            c[0] = intri[0]*colorw/2/intri[2];//fx
            c[1] = intri[1]*colorh/2/intri[3];//fy
            c[2] = colorw/2;//cx
            c[3] = colorh/2;//cy

            d[0] = intri[0]*depthw/2/intri[2];//fx
            d[1] = intri[1]*depthh/2/intri[3];//fy
            d[2] = depthw/2;//cx
            d[3] = depthh/2;//cy

            Frame::intrinsicD.setIntrinsic(c, d);
        }

        readParameter(paramFileNames[i], frames[i]);
    }
}

void FrameReader::readData(std::string argv1, std::vector<m3d::Frame> &frames)
{
    readArgv(argv1);

    std::string paramDefault;
    std::vector<std::string> imageFileNames, depthFileNames, paramFileNames;

    FileNameExtractor fileNameExtractor(dataDir, imageFileNames, depthFileNames, paramFileNames, paramDefault, state);

    m3d::IntrinsicD intrinDefault;
    readDefaultIntrinsic(paramDefault, intrinDefault);

    readImages(imageFileNames, frames);
    readDepths(depthFileNames, frames);

    readParameters(paramFileNames, intrinDefault, frames);
}

FrameReader::FrameReader(std::string argv1, std::vector<m3d::Frame> &frames)
{
    readData(argv1, frames);
}

FrameReader::~FrameReader() {}