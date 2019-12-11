/* DisjointSet
 * Copyright (C) 2019 Pierre Vigier
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <vector>

namespace ds
{

class DisjointSets
{
public:
    DisjointSets(std::size_t size) : mNodes(size), mNbSets(size)
    {
        for (auto i = std::size_t(0); i < mNodes.size(); ++i)
        {
            auto& node = mNodes[i];
            node.parent = i;
            node.rank = 0;
            node.size = 1;
        }
    }

    std::size_t find(std::size_t x) const
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

    void unite(std::size_t x, std::size_t y)
    {
        auto& rootX = mNodes[find(x)];
        auto& rootY = mNodes[find(y)];
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

    bool same(std::size_t x, std::size_t y) const
    {
        return find(x) == find(y);
    }

    std::size_t getSize() const
    {
        return mNodes.size();
    }

    std::size_t getSetSize(std::size_t x) const
    {
        return mNodes[find(x)].size;
    }

    std::size_t getNbSets() const
    {
        return mNbSets;
    }

private:
    struct Node
    {
        std::size_t parent;
        std::size_t rank;
        std::size_t size;
    };

    mutable std::vector<Node> mNodes;
    std::size_t mNbSets;
};

}