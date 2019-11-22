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
        double rts[6];//3 dim angle axis rotation and 3dim translation
        double matches[2];//matches_inliners, matches_all
        double translations[2]; // delta_R, delta_t, delta越小说明越可靠
        std::vector<cv::KeyPoint> srcPts, dstPts;
    };

    class Graph
    {
    private:

    public:
        std::vector<Edge> edges;
        std::vector<bool> activated;
        m3d::DisjointSets disjointSets;

        Graph(std::size_t sz = 0)
        : disjointSets(sz)
        {
            edges.clear();
            activated.clear();

            if(sz>0)
                activated.resize(sz, false);
        }

        void resize(std::size_t sz)
        {
            edges.clear();
            activated.clear();
            activated.resize(sz);
            disjointSets.resize(sz);
        }

        void addEdge(Edge edge)
        {
            edges.push_back(edge);
        }
    };



}


#endif //MY3DPHOTO_0GRAPH_H
