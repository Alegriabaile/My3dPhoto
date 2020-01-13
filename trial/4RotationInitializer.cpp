//
// Created by ale on 20-1-10.
//
#include "1Logger.h"
#include "4RotationInitializer.h"
#include "GCoptimization.h"

#include <iostream>
#include <Eigen/Eigen>
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

static class ExtraData
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
        const int midH = height/2;
        const int midW = width/2;
        for(size_t h = 0; h < height; ++h)
        {
            for(size_t w = 0; w < width; ++w)
            {
                float normH = (float)(h - midH)*2.0/(float)height;
                float normW = (float)(w - midW)*2.0/(float)width;
                //Edata = p(|| normH, normW ||*100, 100).
                unaryCosts[h*width + w] = std::min((int)((std::pow( normH, 2) + std::pow( normW, 2))*100.0), 100);
            }
        }

        //debug...
        cv::Mat unaryImage(height, width, CV_8UC1, cv::Scalar(0));
        for(size_t h = 0; h < height; ++h)
            for(size_t w = 0; w < width; ++w)
                unaryImage.at<uchar>(h, w) = unaryCosts[h*width + w];

        cv::imshow("unaryImage", unaryImage);
        cv::waitKey();
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

static class ExtraSmoothData
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
                xs[h*width + w] = std::sin(h*L2A_H)*std::cos(w*L2A_W);
                ys[h*width + w] = std::sin(h*L2A_H)*std::sin(w*L2A_W);
                zs[h*width + w] = std::cos(h*L2A_H);
            }
        }

        //debug...
        cv::Mat binaryImage(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
        for(size_t h = 0; h < height; ++h)
        {
            for(size_t w = 0; w < width; ++w)
            {
                const float x = xs[h*width + w];
                const float y = ys[h*width + w];
                const float z = zs[h*width + w];
                int b = (x + 1.0)/2.0*255;
                int g = (y + 1.0)/2.0*255;
                int r = (z + 1.0)/2.0*255;
                //front and back.
                if(std::fabs(x) >= std::fabs(y) && std::fabs(x) >= std::fabs(z))
                    g = r = 0;
                else if(std::fabs(y) >= std::fabs(z))
                    b = r = 0;
                else
                    b = g = 0;

                binaryImage.at<cv::Vec3b>(h, w) = cv::Vec3b(b, g, r);
            }
        }
        cv::imshow("binaryImage", binaryImage);
        cv::waitKey();

        //todo with rodrigues-to-rotations...
        mRotationMats = std::vector<cv::Mat>(length*length, cv::Mat());
        for(int h = 0; h < length; ++h)
            for(int w = 0; w < length; ++w)
            if(mMatPedges[h*length + w] != NULL)
            {
                double *relativePose = mMatPedges[h*length + w]->rts;
                cv::Mat rVec = (cv::Mat_<double>(3, 1) << relativePose[0], relativePose[1], relativePose[2]);
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
        //todo
        //3. how to define the distance function( difference of relative rot, or difference of the effect of relative rot)?
        //min(|| vj(xj, yj, zj) - Rij*vi(xi, yi, zi) ||, 1).
        cv::Mat vi, vj;
//        if(mMatPedges[p1*length + p2]->src == mDense2Sparse[p1])
//        {
//            vi = (cv::Mat_<double>(3, 1) << xs[l1], ys[l1], zs[l1]);
//            vj = (cv::Mat_<double>(3, 1) << xs[l2], ys[l2], zs[l2]);
//        }else
//        {
//            vi = (cv::Mat_<double>(3, 1) << xs[l2], ys[l2], zs[l2]);
//            vj = (cv::Mat_<double>(3, 1) << xs[l1], ys[l1], zs[l1]);
//        }
//
//        //Quaternion should be faster...
//        double *relativePose = mMatPedges[p1*length + p2]->rts;
//        cv::Mat rVec = (cv::Mat_<double>(3, 1) << relativePose[0], relativePose[1], relativePose[2]);
//        cv::Mat R = cv::Mat(3,3,CV_64FC1);
//        cv::Rodrigues(rVec, R);

        vi = (cv::Mat_<double>(3, 1) << xs[l1], ys[l1], zs[l1]);
        vj = (cv::Mat_<double>(3, 1) << xs[l2], ys[l2], zs[l2]);
        cv::Mat & R = mRotationMats[p1*length + p2];
        cv::Mat error = vj - R*vi;

        //Esmooth = min(|| vj(xj, yj, zj) - Rij*vi(xi, yi, zi) ||*100.0, 100).
        return std::min((int)(cv::norm(error)*100), 100);
    }
};

static int dataCostFunction(int p, int l, void *data)
{
    ExtraData * extraData = (ExtraData*)data;
    return extraData->getDataCost(p, l);
}
static int smoothCostFunction(int p1, int p2, int l1, int l2, void *extraData)
{
    ExtraSmoothData * extraSmoothData = (ExtraSmoothData *)extraData;
    return extraSmoothData->getSmoothCost(p1, p2, l1, l2);
}



//////////////////////////////////////////////////////////////////////////////////////
/**************************** *******************************/
/**************************** implemention of class RotationInitializer *******************************/
/**************************** *******************************/
//////////////////////////////////////////////////////////////////////////////////////
namespace m3d
{

    RotationInitializer::RotationInitializer(std::vector<m3d::Frame> &frames_, m3d::Graph &graph_)
            :frames(frames_), graph(graph_)
    {
        ActivateEdges();
        GenInitialRotations();
    }
    RotationInitializer::~RotationInitializer(){}


    bool RotationInitializer::IsActiveEdge(const m3d::Edge &edge)
    {
        //more reliable judgement methods needed...
        //todo

        //to run the dataset for MI-3d-photo-project
        double gmsInliersNo = edge.matches[0];
        double allPairsNo = edge.matches[1];

        if(gmsInliersNo < 100)
            return false;

        double deltaRotation = edge.translations[0];//Delta Rotation of angle-axis.
        if(deltaRotation < 0.1 || deltaRotation > 0.5)
            return false;

        size_t inliersOfRecoverPose = edge.srcPts.size();
        if(inliersOfRecoverPose < 50)
            return false;

        return true;
    }

    void RotationInitializer::ActivateEdges()
    {
        std::vector<Edge> &edges = graph.edges;
        std::vector<bool> &activated = graph.activatedEdges;
        DisjointSets &disjointSets = graph.disjointSets;

        size_t sz = edges.size();
        for(size_t i = 0; i < sz; ++i)
            activated[i] = IsActiveEdge(edges[i]);
    }

    void RotationInitializer::GenInitialRotations()
    {
        std::vector<Edge> &edges = graph.edges;
        std::vector<bool> &activatedEdges = graph.activatedEdges;

        std::vector<bool> &activatedFrames = graph.activatedFrames;
        DisjointSets &disjointSets = graph.disjointSets;

        //disjoint set, cut the frames to connected graphs
        size_t esz = edges.size();
        for(size_t i = 0; i<esz; ++i)
            if(activatedEdges[i])
                disjointSets.Union(edges[i].src, edges[i].dst);

        //find the max graph(decided by vertices/frames).
        size_t parentOfDisjointSet = disjointSets.getMaxSetParent();
        size_t vsz = activatedFrames.size();

        //
        for(size_t i=0; i<vsz; ++i)
            if(disjointSets.Find(i) == parentOfDisjointSet)
                activatedFrames[i] = true;

        //generate initial rotations from mrf initializer...
        mDense2Sparse.clear();
        mSparse2Dense.clear();
        for(size_t i = 0; i < vsz; ++i)
        {
            if(activatedFrames[i])
            {
                mDense2Sparse.push_back(i);
                mSparse2Dense[i] = mDense2Sparse.size() - 1;
            }
        }
        if(mDense2Sparse.empty())
        {
            printf("GcoStichingSolver::initialize(): mLabel2Index.empty().\n");
            exit(-1);
        }

        const size_t length = mDense2Sparse.size();

        mVecPframes.clear();
        mVecPframes = std::vector<Frame*>(length, NULL);
        for(size_t i = 0; i < vsz; ++i)
            if(activatedFrames[i])
                mVecPframes[mSparse2Dense[i]] = &frames[i];

        mMatPedges.clear();
        mMatPedges = std::vector<Edge*>(length*length, NULL);
        mEdgeFlags.clear();
        mEdgeFlags = std::vector<bool>(esz, false);
        for(size_t i = 0; i < esz; ++i)
        {
            if(!activatedEdges[i])
                continue;

            size_t src = edges[i].src;
            size_t dst = edges[i].dst;
            if(activatedFrames[src] && activatedFrames[dst])
            {

                mMatPedges[mSparse2Dense[src] + length*mSparse2Dense[dst]] =
                        mMatPedges[mSparse2Dense[dst] + length*mSparse2Dense[src]] = &edges[i];
                mEdgeFlags[i] = true;
            }
        }

        SolveRotationLabelling();
    }

    void RotationInitializer::SolveRotationLabelling()
    {
        size_t width = 360;
        size_t height = 180;
        ExtraData extraData(mVecPframes, height, width);
        ExtraSmoothData extraSmoothData( mDense2Sparse, mSparse2Dense, mMatPedges, height, width, mVecPframes.size());

        size_t num_pixels = mVecPframes.size();
        size_t num_labels = height * width;//segmentation.

        std::vector<size_t > results(num_pixels);
        try{
            GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(num_pixels,num_labels);
            gc->setDataCost(dataCostFunction, &extraData);
            gc->setSmoothCost(smoothCostFunction, &extraSmoothData);

            //set edges of relative pixels.
            for(size_t v1 = 0; v1 < num_pixels - 1; ++v1)
                for(size_t v2 = v1 + 1; v2 < num_pixels; ++v2)
                    if(mMatPedges[v1 + v2*num_pixels] != NULL)
                        gc->setNeighbors( v1, v2);

            printf("\nBefore optimization energy is %lld",gc->compute_energy());
            gc->expansion(100);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
            printf("\nAfter optimization energy is %lld",gc->compute_energy());

            for(size_t i = 0; i < num_pixels; ++i)
                results[i] = gc->whatLabel(i);

            delete gc;
        }
        catch (GCException e){
            e.Report();
        }

        //todo
        //update the resulting labels to initial rotations.
    }

}