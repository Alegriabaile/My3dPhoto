//
// Created by ale on 19-11-18.
//

#ifndef MY3DPHOTO_0DISJOINTSET_H
#define MY3DPHOTO_0DISJOINTSET_H

#include <vector>

namespace m3d
{
    class DisjointSets
    {

    private:
        struct Node
        {
            std::size_t parent;
            std::size_t rank;
            std::size_t size;
        };

        mutable std::vector<Node> mNodes;
        std::size_t mNbSets;

    public:
        void resize(std::size_t size)
        {
            mNodes.resize(size);
            mNbSets = size;
            for (auto i = std::size_t(0); i < mNodes.size(); ++i)
            {
                auto& node = mNodes[i];
                node.parent = i;
                node.rank = 0;
                node.size = 1;
            }
        }
        DisjointSets(std::size_t size)
        {
            resize(size);
        }

        std::size_t Find(std::size_t x) const
        {
            // Find the root
            auto y = x;
            while (mNodes[y].parent != y)
                y = mNodes[y].parent;
            // Path compression
            while (mNodes[x].parent != x)
            {
                auto& node = mNodes[x];
                x = node.parent;
                node.parent = y;
            }

            return y;
        }

        void Union(std::size_t x, std::size_t y)
        {
            auto& rootX = mNodes[Find(x)];
            auto& rootY = mNodes[Find(y)];
            if (rootX.parent != rootY.parent)
            {
                if (rootX.rank < rootY.rank)
                {
                    rootX.parent = rootY.parent;
                    rootY.size += rootX.size;
                }
                else
                {
                    rootY.parent = rootX.parent;
                    rootX.size += rootY.size;
                    if (rootX.rank == rootY.rank)
                        ++rootX.rank;
                }
                --mNbSets;
            }
        }

        std::size_t getSetSize(std::size_t x) const
        {
            return mNodes[Find(x)].size;
        }

        std::size_t getMaxSetParent()
        {
            auto maxSz = std::size_t(0);
            auto maxParent = std::size_t(0);
            for (auto i = std::size_t(0); i < mNodes.size(); ++i)
            {
                std::size_t nodeSz = mNodes[Find(i)].size;
                if(maxSz < nodeSz)
                {
                    maxSz = nodeSz;
                    maxParent = i;
                }
            }

            return maxParent;
        }

    };

}

#endif //MY3DPHOTO_0DISJOINTSET_H
