//
// Created by ale on 19-12-18.
//

#include "9Viewer.h"

void myRun(const std::string &argv1);

int main(int argc, char** argv)
{
    if(argc>2)
        myRun( argv[1]);
    else
    {
        myRun( "argv.txt");
    }


    return 0;
}

void myRun(const std::string &argv1)
{
    std::ifstream inFile(argv1);
    if (!inFile) {
        printf("FrameReader::readArgv:: argv file does not exist.");
        exit(-1);
    }
    std::string dataDir;
    size_t state;
    inFile >> dataDir >> state;
    inFile.close();

    m3d::Viewer viewer(dataDir);
}