//
// Created by ale on 20-1-15.
//

#ifndef MY3DPHOTO_4ROTATIONCOSTS_H
#define MY3DPHOTO_4ROTATIONCOSTS_H

#include <iostream>
//////////////////////////////////////////////////////////////////////////////////////
/**************************** *******************************/
/**************************** gco-lib c-type solver... *******************************/
/**************************** *******************************/
//////////////////////////////////////////////////////////////////////////////////////

//#modelling: 1st version.
//Q1. using the (pan, tilt, twist) to describe the rotation, and "twist" is zero, encoded with Spherical Coordinates with nor twist(180*360 instead of 11*11*11).
//Q2. smoothness term: Esmooth = p(||  vj - Rij*vi  ||) instead of p(||  vij - R0(vi)-1*vj  ||.
//Q3. Representation from spherical coordinate to angle-axis, and reversing: Euler-angles to angle-axis. Euler-angles is a special case of angle-axis.
//    if IMU exist, add distance, else add constraints(ensure them align in the center of panorama).

//#details.
//    Q1: in the standard [Spherical coordinate system]<-->[Cartesian coordinates system], x=rsinθcosφ, y=rsinθsinφ, z=rcosθ, and
//        r=sqrt(x^2 + y^2 + z^2), θ=arccos(z/r), φ=arctan(y/x).  r = 1.
//    Q2: min(|| vj(xj, yj, zj) - Rij*vi(xi, yi, zi) ||, 1).
//    Q3: Euler-angles to represent spherical coordinates of "pan + tilt". Angleaxis(angle, axis) to represent the euler-angles.


//h, w: h[0.5, h-0.5], w[1, w].
class ExtraData
{
    int * unaryCosts;
    void Initialize()
    {
        unaryCosts = new int[height*width];
        if(unaryCosts == NULL)
        {
            printf("ExtraData::ExtraData(): unaryCosts is NULL!!!\n");
            exit(-1);
        }
        const float midH = height/2.0;
        const float midW = width/2.0;

        printf("unary data: \n");
        for(int h = 0; h < height; ++h)
        {
            for(int w = 0; w < width; ++w)
            {
//                if(h == 0 || w == 0 || h == height - 1 || w == width - 1)
//                {
//                    unaryCosts[h*width + w] = 10000;
//                    continue;
//                }
                //in the middle of pano-center. θ:[pi/4-3*pi/4], φ:[pi/2, 3*pi/2].
                float normH = fabs((float)(h - midH + 0.5)*2.0/(float)height);
                float normW = fabs((float)(w - midW + 1.0)/(float)width);
                //Edata = p(|| normH, normW ||*100, 100).
                //unaryCosts[h*width + w] = std::min((int)((std::pow( normH, 2) + std::pow( normW, 2))*120.0), 100);
                float Edata = std::pow(std::max(normH, normW), 2);
                //unaryCosts[h*width + w] = std::min((int)(Edata*10), 20);
                unaryCosts[h*width + w] = (int)(Edata*1000.0);

                printf("%f  ", Edata);
            }
            printf("\n");
        }
    }

public:
    size_t height, width;
    std::vector<m3d::Frame*> & mVecPframes;
    ExtraData( std::vector<m3d::Frame*> & mVecPframes_, size_t height_, size_t width_)
            : mVecPframes(mVecPframes_), height(height_), width(width_)
    {
        Initialize();
    }

    ~ExtraData()
    {
        if(unaryCosts != NULL)
            delete [] unaryCosts;
    }

    //int p: index of frames.
    //int l: index of label(rotations).
    int getDataCost(int p, int l)
    {
        //if mVecPframes[i]->IMU is not empty,
        // todo
        //else
        return unaryCosts[l];
    }
};

class ExtraSmoothData
{
    void Initialize()
    {
        xs = new float[height*width];
        ys = new float[height*width];
        zs = new float[height*width];
        if( xs == NULL || ys == NULL || zs == NULL)
        {
            printf("ExtraData::Initialize(): xs == NULL || ys == NULL || zs == NULL !!!\n");
            exit(-1);
        }

        //x=rsinθcosφ, y=rsinθsinφ, z=rcosθ, θ: related to h, φ: related to w.
        const float L2A_W = 2.0*M_PI/(float)width;
        const float L2A_H = M_PI/(float)height;
        for(size_t h = 0; h < height; ++h)
        {
            for(size_t w = 0; w < width; ++w)
            {
                xs[h*width + w] = std::sin((h)*L2A_H)*std::cos((w)*L2A_W);
                ys[h*width + w] = std::sin((h)*L2A_H)*std::sin((w)*L2A_W);
                zs[h*width + w] = std::cos((h)*L2A_H);
            }
        }

        for(size_t h = 0; h < height; ++h)
        {
            for(size_t w = 0; w < width; ++w)
            {
                const float x = xs[h*width + w];
                const float y = ys[h*width + w];
                const float z = zs[h*width + w];
            }
        }

        //todo with rodrigues-to-rotations...
        mRotationMats = std::vector<cv::Mat>(length*length, cv::Mat());
        for(int h = 0; h < length; ++h)
            for(int w = 0; w < length; ++w)
                if(mMatPedges[h*length + w] != NULL)
                {
                    double *rts = mMatPedges[h*length + w]->rts;
                    cv::Mat rVec = (cv::Mat_<double>(3, 1) << rts[0], rts[1], rts[2]);
                    cv::Mat R = cv::Mat(3,3,CV_64FC1);
                    cv::Rodrigues(rVec, R);
                    if(mMatPedges[h*length + w]->src != mDense2Sparse[h])
                        cv::transpose(R, R);//input-output could be the same mat...
                    mRotationMats[h*length + w] = R.clone();
                }
    }

public:
    size_t height, width, length;
    float *xs, *ys, *zs;
    std::vector<cv::Mat> mRotationMats;
    std::vector<size_t> &mDense2Sparse;
    std::map<size_t, size_t> &mSparse2Dense;
    std::vector<m3d::Edge*> & mMatPedges;

    ExtraSmoothData( std::vector<size_t> &mDense2Sparse_, std::map<size_t, size_t> &mSparse2Dense_,
                     std::vector<m3d::Edge*> & mMatPedges_, size_t height_, size_t width_, size_t length_)
            : mDense2Sparse(mDense2Sparse_), mSparse2Dense(mSparse2Dense_), mMatPedges(mMatPedges_), height(height_), width(width_), length(length_)
    {
        Initialize();
    }
    ~ExtraSmoothData()
    {
        if(xs != NULL)
            delete [] xs;
        if(ys != NULL)
            delete [] ys;
        if(zs != NULL)
            delete [] zs;
        xs = ys = zs = NULL;
    }

    //int p: index of frames.
    //int l: index of label(rotations).
    int getSmoothCost(int p1, int p2, int l1, int l2)
    {
        //min(|| vj(xj, yj, zj) - Rij*vi(xi, yi, zi) ||, 1).
        cv::Mat vi, vj;
        vi = (cv::Mat_<double>(3, 1) << xs[l1], ys[l1], zs[l1]);
        vj = (cv::Mat_<double>(3, 1) << xs[l2], ys[l2], zs[l2]);
        cv::Mat & R = mRotationMats[p1*length + p2];
        cv::Mat error = vi - R*vj;

        //Esmooth = min(|| vj(xj, yj, zj) - Rij*vi(xi, yi, zi) ||*1000.0, 1000).
        int Esmooth = (int)(cv::norm(error)*10000.0);
//        printf("p1,p2,l1,l2:%d,%d,%d,%d:%d.  ", p1, p2, l1, l2, Esmooth);
        //return std::min(Esmooth, 1000);
        return Esmooth;
    }
};

int dataCostFunction(int p, int l, void *data)
{
    ExtraData * extraData = (ExtraData*)data;
    return extraData->getDataCost(p, l);
}
int smoothCostFunction(int p1, int p2, int l1, int l2, void *extraData)
{
    ExtraSmoothData * extraSmoothData = (ExtraSmoothData *)extraData;
    return extraSmoothData->getSmoothCost(p1, p2, l1, l2);
}

#endif //MY3DPHOTO_4ROTATIONCOSTS_H
