//
// Created by ale on 19-11-7.
//

#ifndef MY3DPHOTO_1FRAMEREADER_H
#define MY3DPHOTO_1FRAMEREADER_H

#include "0Frame.h"
#include "1FileNameExtractor.h"

namespace m3d
{
    class FrameReader
    {
    private:
        std::string dataDir;
        unsigned int state;

        void readArgv(std::string argv1)
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

        void readDefaultIntrinsic(const std::string &paramDefault, m3d::IntrinsicD& intrinsicD)
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
        void readParameters(const std::string &paramFileName, m3d::IntrinsicD &intrinsicD, m3d::ExtrinsicD &extrinsicD)
        {

        }

        void readData(std::string argv1, std::vector<m3d::Frame> &frames)
        {
            readArgv(argv1);

            std::string paramDefault;
            std::vector<std::string> imageFileNames, depthFileNames, paramFileNames;

            FileNameExtractor fileNameExtractor(dataDir, imageFileNames, depthFileNames, paramFileNames, paramDefault);

            m3d::IntrinsicD intrinDefault;
            readDefaultIntrinsic(paramDefault, intrinDefault);



        }

    public:
        FrameReader(std::string argv1, std::vector<m3d::Frame> &frames)
        {
            readData(argv1, frames);
        }


    };




}



#endif //MY3DPHOTO_1FRAMEREADER_H
