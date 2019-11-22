//
// Created by ale on 19-11-22.
//

#ifndef MY3DPHOTO_4GRAPHINITIALIZER_H
#define MY3DPHOTO_4GRAPHINITIALIZER_H

#include "0m3d.h"
#include <set>

namespace m3d
{
    class GraphInitializer
    {
    private:
        std::vector<m3d::Frame> &frames;
        m3d::Graph &graph;

        bool IsActiveEdge(const m3d::Edge &edge);
        void ActivateEdges();

        //Generate minimum spanning tree by Prim's thought.
        void GenMstViaPrim(std::set<size_t> &Vs, std::vector<m3d::Edge> &Es);
        void GenInitViaMst();

    public:
        GraphInitializer(std::vector<m3d::Frame> &frames_, m3d::Graph &graph_);
        ~GraphInitializer();
    };
}

#endif //MY3DPHOTO_4GRAPHINITIALIZER_H
