//
// Created by ale on 19-11-18.
//

#ifndef MY3DPHOTO_0GRAPH_H
#define MY3DPHOTO_0GRAPH_H

namespace m3d
{
    class Edge
    {
    public:

        static const double c;
        static const double k;
        std::size_t src, dst;
        double rts[6];//3 dim angle axis rotation and 3dim translation
        double costs[4];//matches_inliners, matches_all, delta_R, delta_t, delta越小说明越可靠
        std::vector<cv::KeyPoint> srcPts, dstPts;
    };
    const double Edge::c = 100;
    const double Edge::k = 0.5;

    class Graph
    {
    private:

    public:
        std::vector<Edge> edges;
        std::vector<bool> activated;

        Graph(std::size_t sz = 0)
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
        }

        void addEdge(Edge edge)
        {
            edges.push_back(edge);
        }
    };



}


#endif //MY3DPHOTO_0GRAPH_H
