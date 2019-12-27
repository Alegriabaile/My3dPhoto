//
// Created by ale on 19-12-27.
//

#ifndef MY3DPHOTO_7GCOSTITCHINGSOLVER_H
#define MY3DPHOTO_7GCOSTITCHINGSOLVER_H

#include "0m3d.h"

namespace m3d {

    class GcoStichingSolver
    {
    private:
        const std::vector<bool> activatedFrames;
        const std::vector<m3d::Frame> &frames;
        m3d::Frame &result;

        int mWidth, mHeight, mNumLabels;
        int *dataTerms, *smoothTerms;
        GCoptimizationGridGraph *gc;

        std::vector<size_t> mLabel2Index;

        bool initialize();
        void solve(int num_iters = 10);
        void update();
    public:
        GcoStichingSolver(const std::vector<bool> &activatedFrames, const std::vector<m3d::Frame> &frames, m3d::Frame &result);
        ~GcoStichingSolver();
    };


}

#endif //MY3DPHOTO_7GCOSTITCHINGSOLVER_H
