//
// Created by ale on 19-11-6.
//

#include<iostream>

#include"0Frame.h"
#include "1FileNameExtractor.h"
#include "1FrameReader.h"

int main()
{
    using namespace std;
    using namespace m3d;

    string argv("argv.txt");
    vector<Frame> frames;

    FrameReader(argv, frames);

    ifstream infile("argv.txt");
    string path_;
    double no_;
    infile>>path_>>no_;
    cout<<(infile>>path_)<<endl;
    infile.close();

    return 0;
}