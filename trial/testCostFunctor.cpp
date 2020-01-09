//
// Created by ale on 20-1-6.
//

#include "testCostFunctor.h"

void GeneralGraphDfnSfn(const cv::Mat &input, cv::Mat &output)
{
    size_t width = input.cols;
    size_t height = input.rows;
    size_t num_pixels = width*height;
    size_t num_labels = 10;//segmentation.

    output = input.clone();
    ExtraData extraData(input);
    ExtraSmoothData extraSmoothData(input);

    try{
        GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(num_pixels,num_labels);
        gc->setDataCost(dataCostFunction, &extraData);
        gc->setSmoothCost(smoothCostFunction, &extraSmoothData);

        //set edges of relative pixels.
        for(size_t h = 0; h < height; ++h)
            for(size_t w = 0; w < width - 1; ++w)
                gc->setNeighbors(h*width + w, h*width + w + 1);
        for(size_t h = 0; h < height - 1; ++h)
            for(size_t w = 0; w < width; ++w)
                gc->setNeighbors(h*width + w, (h+1)*width + w);

        printf("\nBefore optimization energy is %lld",gc->compute_energy());
        gc->expansion(100);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
        printf("\nAfter optimization energy is %lld",gc->compute_energy());

        for(size_t h = 0; h < height; ++h)
            for(size_t w = 0; w < width; ++w)
                output.at<uchar>(h ,w) = gc->whatLabel(h*width + w)*64;
//                output.at<uchar>(h ,w) = gc->whatLabel(h*width + w)>0?255:0;

        delete gc;
    }
    catch (GCException e){
        e.Report();
    }

}


int main(int argc, char **argv)
{
    cv::Mat input, output;
    input = cv::imread("data/cameraman.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
    if(input.empty())
        exit(-1);
    std::cout<<"before convertTo(input, CV_8UC1):  depth, channels: "<<input.depth()<<", "<<input.channels()<<std::endl;
    input.convertTo(input, CV_8UC1);
    std::cout<<"after convertTo(input, CV_8UC1):  depth, channels: "<<input.depth()<<", "<<input.channels()<<std::endl;

    //do segmentation...
    GeneralGraphDfnSfn(input, output);


    //simple threshold method.
    cv::Mat output_2 = cv::Mat(input.size(), CV_8UC1, cv::Scalar(0));
    output_2.setTo(255, input>128);
    cv::imshow("output_2", output_2);

    //original image.
    cv::imshow("input", input);
    //segmented image.
    cv::imshow("output", output);

    std::cout<<output<<std::endl;
//    cv::waitKey();

    return 0;
}