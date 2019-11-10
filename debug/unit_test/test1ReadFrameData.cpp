//
// Created by ale on 19-11-8.
//

//
// Created by ale on 19-11-6.
//

#include<iostream>

#include "0Frame.h"
#include "1FrameReader.h"

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

    for(int i=0; i<frames.size(); ++i)
    {
        cout<<endl<<"/////////////////////////////     frame[ "<<i<<" ]      ///////////////////////////"<<endl;
        cout<<frames[i].imageFileName<<endl;
        cout<<frames[i].depthFileName<<endl;
        cout<<frames[i].paramFileName<<endl;

        imshow("image", frames[i].image);
        imshow("depth", frames[i].depth);
        imshow("disparity", frames[i].disparity);

        waitKey();

        int x = 0;
    }

    return 0;
}