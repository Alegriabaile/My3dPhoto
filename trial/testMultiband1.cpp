//
// Created by ale on 19-12-27.
//

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <iostream>

using namespace std;
using namespace cv;

// show image
void showImage(String windowName, Mat img, int flag = 0)
{
    namedWindow(windowName, CV_WINDOW_NORMAL);//0: CV_WINDOW_NORMAL 1: WINDOW_AUTOSIZE
    imshow(windowName, img);
    waitKey(0);
    //destroyWindow(windowName);
}

vector<Mat_<float>> gaussianPyramid(Mat img, int levels)
{
    vector<Mat_<float>> _gaussianPyramid;
    _gaussianPyramid.push_back(img);
    Mat currentImage = img;
    //cout << currentImage.size() << endl;
    for (int i = 1; i < levels; i++)
    {
        Mat downsampleImage;
        pyrDown(currentImage, downsampleImage); // Blurs an image and downsamples it.
        _gaussianPyramid.push_back(downsampleImage);
        //showImage("currentImage", currentImage);
        //cout << downsampleImage.size() << endl;
        currentImage = downsampleImage;
    }
    return _gaussianPyramid;
}


vector<Mat_<float>> laplacianPyramid(vector<Mat_<float>> gaussPyrImg)
{
    int levels = gaussPyrImg.size();
    vector<Mat_<float>> _laplacianPyramid;
    _laplacianPyramid.push_back(gaussPyrImg[levels - 1]);// order reverse !!
    //cout << gaussPyrImg[levels - 1].size() << endl;
    for (int i = levels - 2 ; i >= 0; i--)
    {
        Mat upsampleImage;
        pyrUp(gaussPyrImg[i + 1], upsampleImage, gaussPyrImg[i].size());
        Mat currentImage = gaussPyrImg[i] - upsampleImage;
        //showImage("currentImage", currentImage);
        //cout << currentImage.size() << endl;
        _laplacianPyramid.push_back(currentImage);
    }
    return _laplacianPyramid;
}


vector<Mat_<float>> blendPyramid(vector<Mat_<float>> pyrA, vector<Mat_<float>> pyrB, vector<Mat_<float>> pyrMask)
{
    int levels = pyrA.size();
    vector<Mat_<float>> blendedPyramid;
    for (int i = 0; i < levels; i++)
    {
        Mat blendedImage = pyrA[i].mul(1.0 - pyrMask[levels -1 - i]) + pyrB[i].mul(pyrMask[levels - 1 - i]);
        //showImage("blendedImage", blendedImage);
        blendedPyramid.push_back(blendedImage);
    }
    return blendedPyramid;
}

Mat collapsePyramid(vector<Mat_<float>> blendedPyramid)
{
    int levels = blendedPyramid.size();
    Mat currentImage = blendedPyramid[0];
    for (int i = 1; i < levels; i++)
    {
        pyrUp(currentImage, currentImage, blendedPyramid[i].size());
        currentImage += blendedPyramid[i];
        //showImage("currentImage", currentImage);
    }
    Mat blendedImage;
    convertScaleAbs(currentImage, blendedImage, 255.0);
    return blendedImage;
}

int main()
{
    // Read the images, assuming images to be of same size
    Mat A = imread("trial/ghost rider.jpg", IMREAD_COLOR);
    Mat B = imread("trial/nicholas cage.jpg", IMREAD_COLOR);
    //showImage("leftImage", A);
    //showImage("rightImage", B);

    if (A.size() != B.size())
    {
        A = Mat(A, Range::all(), Range(0, A.cols));
        B = Mat(B, Range(0, A.rows), Range(0, A.cols));
    }

    int height = A.rows;
    int width = A.cols;

    // Convert images to grayscale, float
    Mat _imgA, _imgB, imgA, imgB;
    cvtColor(A, _imgA, CV_BGR2GRAY);
    cvtColor(B, _imgB, CV_BGR2GRAY);

    _imgA.convertTo(imgA, CV_32F, 1.0 / 255.0);
    _imgB.convertTo(imgB, CV_32F, 1.0 / 255.0);


    // Create mask
    Mat_<float> mask(height, width, 0.0);
    mask(Range::all(), Range(mask.cols / 2 + 2, mask.cols )) = 1.0;
    //showImage("mask", mask * 255);

    // Create gaussian pyramids for both the images and the mask
    int levels = floor(log2(min(width, height)));

    vector<Mat_<float>> gaussPyrA = gaussianPyramid(imgA, levels);
    vector<Mat_<float>> gaussPyrB = gaussianPyramid(imgB, levels);
    vector<Mat_<float>> gaussPyrMask = gaussianPyramid(mask, levels);

    // Create Laplacian pyramids from both the images
    vector<Mat_<float>> laplacePyrA = laplacianPyramid(gaussPyrA);
    vector<Mat_<float>> laplacePyrB = laplacianPyramid(gaussPyrB);

    // Blend laplacian pyramid
    vector<Mat_<float>> blendedPyr = blendPyramid(laplacePyrA, laplacePyrB, gaussPyrMask);

    for(size_t i = 0; i < levels; ++i)
    {
        imshow("gaussPyrA", gaussPyrA[i]);
        imshow("gaussPyrB", gaussPyrB[i]);
        imshow("gaussPyrMask", gaussPyrMask[i]);

        imshow("laplacePyrA", laplacePyrA[i]);
        imshow("laplacePyrB", laplacePyrB[i]);
        imshow("blendedPry", blendedPyr[i]);

        printf("gaussPyrA.size(): %d, %d\n", gaussPyrA[i].rows, gaussPyrA[i].cols);
        printf("gaussPyrB.size(): %d, %d\n", gaussPyrB[i].rows, gaussPyrB[i].cols);
        printf("gaussPyrMask.size(): %d, %d\n", gaussPyrMask[i].rows, gaussPyrMask[i].cols);

        printf("laplacePyrA.size(): %d, %d\n", laplacePyrA[i].rows, laplacePyrA[i].cols);
        printf("laplacePyrB.size(): %d, %d\n", laplacePyrB[i].rows, laplacePyrB[i].cols);
        printf("blendedPry.size(): %d, %d\n", blendedPyr[i].rows, blendedPyr[i].cols);
        waitKey();
    }

    // Reconstruct image from blended pyramid
    Mat blendedImg = collapsePyramid(blendedPyr);
    showImage("blendedImg", blendedImg);

    return 0;
}
