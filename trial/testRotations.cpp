//
// Created by ale on 20-1-16.
//
#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

using namespace Eigen;

float rts[] = {
        0.062855, -0.138750, 0.077138, -0.923145, -0.163507, -0.347948,
        0.580208, -0.397980, -0.101111, -0.020586, 1.376434, 1.263354,
        0.571266, 0.143034, 0.171197, 0.215554, 1.417488, 0.009583,
        0.186464, -0.746569, 0.017129, -1.749209, 2.643651, 0.243862,
        0.534530, -0.079165, 0.046622, 0.853859, 1.408928, 0.779319,
        0.035008, -0.788823, -0.048361, -2.196589, 3.448887, 0.633024,
        0.405041, 0.223483, 0.087905, -0.119810, 0.565702, -0.078397,
        -0.139855, -0.383432, 0.039594, -0.967989, 0.248795, 0.033144,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.405138, -0.715726, 0.117021, -1.637219, 1.743479, 0.664753,
        0.372407, -0.409322, 0.077313, -0.702828, 2.010929, 0.900088,
        0.179705, 0.302642, 0.020338, 0.671678, 0.328185, -0.638855,
        0.284641, -0.072069, -0.007367, 0.788564, 0.514632, 0.336632};
int rtsi[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

float rel_rts[] = {
        0.062855, -0.138750, 0.077138, -0.923145, -0.163507, -0.347948,
        0.039664, -0.343069, -0.058191, -0.829263, 0.274282, 0.486922,
        0.167080, -0.035757, -0.210484, 0.771317, -0.567733, 0.287662,
        0.048393, 0.242402, 0.059463, -0.702824, -0.341327, -0.624127,
        0.160002, 0.044785, 0.001617, 0.084723, -0.809577, -0.580867,
        -0.234505, -0.029007, -0.010022, -0.390922, 0.716668, -0.577553,
        0.251690, 0.000299, 0.044883, 0.080511, 0.982432, 0.168362,
        0.124078, 0.304087, 0.047705, -0.935948, -0.064884, -0.346108,
        -0.139855, -0.383432, 0.039594, -0.967989, 0.248795, 0.033144,
        -0.284641, 0.072069, 0.007367, -0.801095, -0.586152, -0.121134,
        0.038590, -0.293992, 0.088091, -0.949863, -0.190202, 0.248160,
        -0.103402, 0.374416, -0.022011, -0.181609, -0.452548, -0.873051};

int srcs[] = { 1, 2, 2, 3, 4, 4, 5, 7, 8, 9, 10, 12,};
int dsts[] = { 9, 5, 11, 5, 6, 10, 13, 13, 9, 13, 11, 13};

int main()
{
    Eigen::Vector3f vi(0, 0, 1);
    Eigen::Vector3f vj(-1, 0, 0);

    Eigen::Matrix3f m;
    m = Eigen::AngleAxisf(-M_PI/2.0, Eigen::Vector3f::UnitY());
    std::cout<<m*vi<<std::endl;

    //src, dst: given a point p(x, y, z), psrc = Rsrc*p + tsrc, pdst = Rdst*p + tdst;
    //relative rts: psrc' = Rrel*pdst + trel, cost = f(psrc, psrc').
    std::cout<<std::endl;
    //i: index of relative transforms...
    for(int i = 0; i < sizeof(srcs)/sizeof(int); ++i)
    {
        int src = srcs[i] - 1;
        int dst = dsts[i] - 1;
        //Eigen::Vector3f dst3(rts[dst*6], rts[dst*6 + 1], rts[dst*6 + 2]);
        //Eigen::Vector3f src3(rts[src*6], rts[src*6 + 1], rts[src*6 + 2]);

        cv::Mat src3 = (cv::Mat_<double>(3, 1) << rts[src*6], rts[src*6 + 1], rts[src*6 + 2]);
        cv::Mat srct = (cv::Mat_<double>(3, 1) << rts[src*6 + 3], rts[src*6 + 4], rts[src*6 + 5]);
        cv::Mat dst3 = (cv::Mat_<double>(3, 1) << rts[dst*6], rts[dst*6 + 1], rts[dst*6 + 2]);
        cv::Mat dstt = (cv::Mat_<double>(3, 1) << rts[dst*6 + 3], rts[dst*6 + 4], rts[dst*6 + 5]);

        cv::Mat srcR, dstR;
        cv::Rodrigues(src3, srcR);
        cv::Rodrigues(dst3, dstR);

        cv::Mat rel3 = (cv::Mat_<double>(3, 1) << rel_rts[i*6], rel_rts[i*6 + 1], rel_rts[i*6 + 2]);
        cv::Mat relt = (cv::Mat_<double>(3, 1) << rel_rts[i*6 + 3], rel_rts[i*6 + 4], rel_rts[i*6 + 5]);
        cv::Mat relR;
        cv::Rodrigues(rel3, relR);

        //std::cout<<"srcR, dstR, relR: " <<std::endl<< srcR<<std::endl<<dstR<<std::endl<<relR<<std::endl;
        //std::cout<<"relR*srcR: "<<std::endl<< relR*srcR<<std::endl<<std::endl;

        //debug......
        //f(a, b) = vi - Rij*vj.
        cv::Mat srcP3 = (cv::Mat_<double>(3, 1) <<0, 0, 1);
        srcP3 = srcR * srcP3;// + srct;
        cv::Mat dstP3 = (cv::Mat_<double>(3, 1) <<0, 0, 1);
        dstP3 = dstR * dstP3;// + dstt;

        cv::Mat srcP3_ = relR * dstP3;// + relt;
        cv::Mat relR_inv;
        cv::transpose(relR, relR_inv);
        cv::Mat dstP3_ = relR_inv * (srcP3);// - relt);

        cv::transpose(srcP3, srcP3);
        cv::transpose(dstP3, dstP3);
        cv::transpose(srcP3_, srcP3_);
        cv::transpose(dstP3_, dstP3_);
        std::cout<<"srcP3, dstP3, srcP3_, dstP3_: "<<std::endl <<
            srcP3<<std::endl<< dstP3<<std::endl<< srcP3_<<std::endl<< dstP3_<<std::endl;

        //debug......
        //f(a, b) = Rij - R(vi)-1*vj.
        cv::Mat srcR_inv, dstR_inv;
        cv::transpose(srcR, srcR_inv);
        cv::transpose(dstR, dstR_inv);
        std::cout<<"Rij, R(vi)-1*Rj, R(vj)-1*Ri: "<<std::endl
            <<relR<<std::endl<<srcR_inv*dstR<<std::endl<<dstR_inv*src<<std::endl;
    }
    std::cout<<std::endl;

}