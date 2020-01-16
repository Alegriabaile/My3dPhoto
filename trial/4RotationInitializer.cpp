//
// Created by ale on 20-1-10.
//
#include "1Logger.h"
#include "4RotationInitializer.h"
#include "4RotationCosts.h"
#include "GCoptimization.h"

#include <iostream>
#include <Eigen/Eigen>
using namespace Eigen;
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

    void RotationInitializer::DiscreteLabel2Rodrigues(
        std::vector<size_t> &labels, size_t height, size_t width, std::vector<m3d::Frame *> &pFrames)
    {
        const size_t sz = labels.size();//results.
        const float L2A_W = 2.0*M_PI/(float)width;
        const float L2A_H = M_PI/(float)height;
        for(size_t k = 0; k < sz; ++k)
        {
            size_t label = labels[k];
            double * rts = pFrames[k]->extrinsicD.rts;

            //x=rsinθcosφ, y=rsinθsinφ, z=rcosθ, θ: related to h, φ: related to w.
            int h = label/width;
            int w = label%width;

            //#  ^z
            //# /
            //#o－－ >x
            //#|
            //#y
            //#################(0, 0, 1) is the original vector.
            //X first then Y.
            float theta = M_PI*0.5 - (h + 0.5)*L2A_H;//rotate X
            float phi = M_PI - (w + 0.5)*L2A_W;//rotate Y

            Eigen::Matrix3f m;
            m = Eigen::AngleAxisf(phi,  Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf angleAxisf( m);//todo
            printf("k, label, h, w: %d, %d, %d, %d.\n", (int)k, (int)label,  h, w);
            Eigen::Vector3f r = angleAxisf.angle()*angleAxisf.axis();
            Eigen::Vector3f t = m * Vector3f(0, 0, 1);
            for(int i=0; i < 3; ++i)
            {
                rts[i] = r[i];
                rts[i+3] = t[i];
            }
        }
    }

    void RotationInitializer::SolveRotationLabelling()
    {
        size_t height = 11;
        size_t width = 37;
        ExtraData extraData(mVecPframes, height, width);
        ExtraSmoothData extraSmoothData( mDense2Sparse, mSparse2Dense, mMatPedges, height, width, mVecPframes.size());

        size_t num_pixels = mVecPframes.size();
        size_t num_labels = height * width;//segmentation.

        std::vector<size_t > results(num_pixels);

        try{
            GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph( num_pixels, num_labels);
            gc->setDataCost(dataCostFunction, &extraData);
            gc->setSmoothCost(smoothCostFunction, &extraSmoothData);

            //set edges of relative pixels.
            for(size_t v1 = 0; v1 < num_pixels - 1; ++v1)
                for(size_t v2 = v1 + 1; v2 < num_pixels; ++v2)
                    if(mMatPedges[v1 + v2*num_pixels] != NULL)
                        gc->setNeighbors( v1, v2);

            printf("\nBefore optimization energy is %lld\n",gc->compute_energy());
            gc->setVerbosity(1);
//            gc->expansion(10);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
//            gc->swap(5);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
            gc->swap(5);
            printf("\nAfter optimization energy is %lld\n",gc->compute_energy());

            for(size_t i = 0; i < num_pixels; ++i)
                results[i] = gc->whatLabel(i);
            delete gc;
        }
        catch (GCException e){
            e.Report();
        }

        //todo
        //update the resulting labels to initial rotations.
        DiscreteLabel2Rodrigues(results, height, width, mVecPframes);
    }
}