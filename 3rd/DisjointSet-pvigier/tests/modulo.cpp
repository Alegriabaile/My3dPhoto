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

#include <cassert>
#include <algorithm>
#include <DisjointSet/DisjointSets.h>

using namespace ds;

int main(int argc, char* argv[])
{
    assert(argc == 3);
    auto size = static_cast<std::size_t>(std::atoi(argv[1]));
    auto modulo = static_cast<std::size_t>(std::atoi(argv[2]));

    auto disjointSets = DisjointSets(size);
    assert(disjointSets.getSize() == size);
    for (auto i = std::size_t(0); i < size; ++i)
        disjointSets.unite(i % modulo, i);
    for (auto i = std::size_t(0); i < size; ++i)
    {
        assert(disjointSets.same(i % modulo, i));
        assert(disjointSets.find(i) == i % modulo);
        assert(disjointSets.getSetSize(i) == size / modulo + (size % modulo > i % modulo ? 1 : 0));
    }
    assert(disjointSets.getNbSets() == modulo);

    return 0;
}