//
// Created by ale on 19-11-7.
//

#ifndef MY3DPHOTO_1FRAMEREADER_H
#define MY3DPHOTO_1FRAMEREADER_H

#include "0m3d.h"
#include "1FileNameExtractor.h"

namespace m3d
{
    class FrameReader
    {
    private:
        std::string dataDir;
        unsigned int state;

        void readArgv(std::string argv1);

        void readDefaultIntrinsic(const std::string &paramDefault, m3d::IntrinsicD& intrinsicD);

        void readImage(const std::string& imageFileName, cv::Mat& image);
        void readImages(const std::vector<std::string>& imageFileNames, std::vector<m3d::Frame> &Frames);

        void readDepth(const std::string& depthFileName, m3d::Frame &frame);
        void readDepths(const std::vector<std::string>& depthFileNames, std::vector<m3d::Frame> &Frames);

        void readParameter(const std::string &paramFileName, m3d::Frame &frame);
        void readParameters(const std::vector<std::string>& paramFileNames,  m3d::IntrinsicD &defaultIntrinsicD, std::vector<m3d::Frame> &Frames);

        void readData(std::string argv1, std::vector<m3d::Frame> &frames);
        void readData(const std::string rootdir, const size_t state, std::vector<m3d::Frame> &frames);
        
    public:
        FrameReader(std::string argv1, std::vector<m3d::Frame> &frames);
        FrameReader(const std::string rootdir_, const size_t state_, std::vector<m3d::Frame> &frames);
        
        ~FrameReader();

    };




}



#endif //MY3DPHOTO_1FRAMEREADER_H
