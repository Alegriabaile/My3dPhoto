//
// Created by ale on 19-11-18.
//

#ifndef MY3DPHOTO_0GRAPH_H
#define MY3DPHOTO_0GRAPH_H

#include "0DisjointSet.h"
namespace m3d
{
    class Edge
    {
    public:
        static const double c;
        static const double k;
        std::size_t src, dst;

        //dst = T*src, or P-dst = R*P-src + s*t;
        //R: Rotation matrix, s: scale ambiguity, t: normalized translation.
        double rts[6];//3 dim angle axis rotation and 3dim translation
        double matches[2];//matches_inliners, matches_all
        double translations[2]; // delta_R, delta_t, delta越小说明越可靠
        std::vector<cv::KeyPoint> srcPts, dstPts;

        Edge(){
            for(size_t i=0; i<6; ++i)
                rts[i] = 0;
            for(size_t i=0; i<2; ++i)
            {
                matches[i] = 0;
                translations[i] = 0;
            }
        }
    };

    class Graph
    {
    private:

    public:
        std::vector<Edge> edges;
        std::vector<bool> activatedEdges;
        std::vector<bool> activatedEdgesMst;
        std::vector<bool> activatedFrames;
        m3d::DisjointSets disjointSets;

        Graph(std::size_t sz = 0)
        : disjointSets(sz), activatedFrames(sz, false)
        {
            edges.clear();
            activatedEdges.clear();
            activatedEdgesMst.clear();
        }

        //resize the sz as frames.size();
        //edges: addEdge, sz*(sz-1)
        void resize(std::size_t sz)
        {
            edges.clear();
            activatedEdges.clear();
            activatedEdgesMst.clear();

            activatedFrames.resize(sz, false);
            disjointSets.resize(sz);
        }

        void addEdge(Edge edge, bool activeEdge = false)
        {
            edges.push_back(edge);
            activatedEdges.push_back(activeEdge);
            activatedEdgesMst.push_back(false);
        }
    };



}


#endif //MY3DPHOTO_0GRAPH_H
