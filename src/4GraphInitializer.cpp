//
// Created by ale on 19-11-22.
//

#include "1Logger.h"
#include "4GraphInitializer.h"


//using namespace cv;
//using namespace std;

namespace m3d
{

    GraphInitializer::GraphInitializer(std::vector<m3d::Frame> &frames_, m3d::Graph &graph_)
    :frames(frames_), graph(graph_)
    {
        ActivateEdges();
        GenInitViaMst();
    }
    GraphInitializer::~GraphInitializer(){}


    bool GraphInitializer::IsActiveEdge(const m3d::Edge &edge)
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

        return true;
    }

    void GraphInitializer::ActivateEdges()
    {
        std::vector<Edge> &edges = graph.edges;
        std::vector<bool> &activated = graph.activatedEdges;
        DisjointSets &disjointSets = graph.disjointSets;

        size_t sz = edges.size();
        for(size_t i = 0; i < sz; ++i)
            activated[i] = IsActiveEdge(edges[i]);
    }

    void GraphInitializer::GenMstViaPrim(std::set<size_t> &Vs, std::vector<m3d::Edge> &Es)
    {
        //sort Edges.
        size_t sz = Es.size();
        if(sz == 0)
        {
            LOG("GraphInitializer::GenMstViaPrim(): ", "Es.size() == 0");
            exit(-1);
        }

        for(size_t i = 0; i< sz - 1; ++i)
            for(size_t j = 0; j < sz - i; ++j)
                if(Es[i].translations[0] > Es[i+1].translations[0])
                    std::swap(Es[i], Es[i+1]);

        //Prim's algorithm.
        std::set<size_t > Vs1(Vs);
        //todo
    }

    void GraphInitializer::GenInitViaMst()
    {
        std::vector<Edge> &edges = graph.edges;
        std::vector<bool> &activatedEdges = graph.activatedEdges;

        std::vector<bool> &activatedFrames = graph.activatedFrames;
        DisjointSets &disjointSets = graph.disjointSets;

        //disjoint set, cut the frames to connected graphs
        size_t sz;
        sz = edges.size();
        for(size_t i = 0; i<sz; ++i)
        {
            if(activatedEdges[i])
            {
                disjointSets.Union(edges[i].src, edges[i].dst);
            }
        }

        //find the max graph(decided by vertices/frames).
        size_t parentOfDisjointSet = disjointSets.getMaxSetParent();
        sz = activatedFrames.size();

        //get the vertices/frames of the max graph.
        std::set<size_t> Vs;
        std::vector<Edge> Es;
        for(size_t i=0; i<sz; ++i)
            if(disjointSets.Find(i) == parentOfDisjointSet)
                Vs.insert(i);

        sz = edges.size();
        for(size_t i=0; i<sz; ++i)
            if(activatedEdges[i])
                Es.push_back(edges[i]);

        GenMstViaPrim(Vs, Es);
    }

}