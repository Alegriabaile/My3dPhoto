//
// Created by ale on 20-1-10.
//

#ifndef MY3DPHOTO_4ROTATIONINITIALIZER_H
#define MY3DPHOTO_4ROTATIONINITIALIZER_H

#include "0m3d.h"

namespace m3d
{
    class RotationInitializer
    {
    private:
        std::vector<m3d::Frame> &frames;
        m3d::Graph &graph;

        std::vector<size_t> mDense2Sparse;
        std::map<size_t, size_t> mSparse2Dense;

        std::vector<bool> mEdgeFlags;
        std::vector<m3d::Frame*> mVecPframes;
        std::vector<m3d::Edge*> mMatPedges;

        bool IsActiveEdge(const m3d::Edge &edge);
        void ActivateEdges();

        void SolveRotationLabelling();
        void GenInitialRotations();

    public:
        RotationInitializer(std::vector<m3d::Frame> &frames_, m3d::Graph &graph_);
        ~RotationInitializer();
    };
}



#endif //MY3DPHOTO_4ROTATIONINITIALIZER_H
