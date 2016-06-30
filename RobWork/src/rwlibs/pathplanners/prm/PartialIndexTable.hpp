/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef RWLIBS_PATHPLANNERS_PRM_PARTIALINDEXTABLE_HPP
#define RWLIBS_PATHPLANNERS_PRM_PARTIALINDEXTABLE_HPP

#include <rw/math/Q.hpp>
#include <list>
#include <vector>
#include <queue>

#include <rw/common/macros.hpp>
#include <rw/models/Device.hpp>
#include <boost/foreach.hpp>

namespace rwlibs { namespace pathplanners { namespace prm {

    /**
     * @brief Provides an Partial Index Table to be used for nearest neighbor search.
     *
     * This class is implemented as a helper for the PRMPlanner.
     */
    template <class T, class Cell = std::list<T> >
    class PartialIndexTable
    {
    public:
        /**
         * @brief Constructs Partial Index Table
         *
         * @param bounds [in] Bounds of the space to partially index
         *
         * @param weights [in] The weights for the WeightedEuclideanMetric used
         * determining whether two nodes are neighbors
         *
         * @param r [in] The distance where two nodes are considered neighbors
         *
         * @param dims [in] The number of dimensions of the table. \b dims has to be
         * within [1, dof]
         */
        PartialIndexTable(
            const rw::models::Device::QBox& bounds,
            rw::math::Q& weights,
            double r,
            int dims)
        {
            RW_ASSERT(0 < dims && dims <= (int)bounds.first.size());

            std::priority_queue<Dimension> queue;
            for (size_t i = 0; i < bounds.first.size(); i++) {
                const double diff = bounds.second(i) - bounds.first(i);
                int divs = (int)std::ceil(diff * weights(i) / r);
                double stepsize = diff / divs;
                Dimension dim = {(int)i, divs, bounds.first(i), stepsize, 0};
                queue.push(dim);
            }

            _tableSize = 0;
            const int n = std::min(dims, (int)queue.size());
            for (int i = 0; i < n; i++) {
                Dimension dim = queue.top();
                queue.pop();
                dim.inc = std::max((int)_tableSize, 1);
                _tableSize = dim.inc*dim.length;
                _dimensions.push_back(dim);
            }
            _table.resize(_tableSize);
        }

        /**
         * @brief Adds a node to the table
         * @param node [in] Node to add
         * @param q [in] Configuration of node
         */
        void addNode(const T& node, const rw::math::Q& q)
        {
            int index = getIndex(q);
            _table[index].push_back(node);
        }

        /**
         * @brief Removes node from the table
         * @param node [in] Node to remove
         *
         * @param q [in] Configuration associated with the node. Used to find the
         * table entry containing the node.
         */
        void removeNode(const T& node, const rw::math::Q& q)
        {
            const int index = getIndex(q);
            Cell& cell = _table[index];

            cell.erase(
                std::remove(cell.begin(), cell.end(), node),
                cell.end());
        }

        /**
         * @brief Searches for all potential neighbors.
         *
         * The potential neighbor is those within the cell associated with \b q and
         * all neighboring cells.
         *
         * To find the true neighbors one has to run through the content of the list
         * and make an exact match.
         *
         * @param q [in] Configuration to search neighbors for
         *
         * @param result [out] The neighbors of \b q
         */
        template <class Collection>
        void searchNeighbors(const rw::math::Q& q, Collection& result) const
        {
            const int index = getIndex(q);
            searchNeighbors(index, _dimensions.begin(), result);
        }

        /**
         * @brief Searches for all potential neighbors.
         *
         * The potential neighbor is those within the cell associated with \b q and
         * all neighboring cells.
         *
         * To find the true neighbors one has to run through the content of the list
         * and make an exact match.
         *
         * @param q [in] Configuration to search neighbors for
         */
        std::list<T> searchNeighbors(const rw::math::Q& q) const
        {
            std::list<T> result;
            searchNeighbors(q, result);
            return result;
        }

    private:
        struct Dimension {
        public:
            int index;
            int length;
            double qoffset;
            double stepsize;
            int inc;
        };

        /**
         * @brief Compares which of two dimensions is the best for the table
         */
        friend bool operator<(const Dimension& a, const Dimension& b)
        {
            return a.length < b.length;
        }

        /**
         * @brief Calculates the index for a given configuration
         */
        int getIndex(const rw::math::Q& q) const
        {
            int index = 0;
            BOOST_FOREACH(const Dimension& dim, _dimensions) {
                int i1 = static_cast<int>(
                    (q(dim.index) - dim.qoffset) / dim.stepsize);

                index += i1 * dim.inc;
            }
            return index;
        }

        template <class Collection>
        void getElements(int index, Collection& result) const
        {
			const Cell& cell = _table[index];
            result.insert(result.end(), cell.begin(), cell.end());
        }

        /**
         * @brief Recursive method running through all the dimensions
         */
        template <class Collection>
        void searchNeighbors(
            int index,
            typename std::vector<Dimension>::const_iterator it,
            Collection& result) const
        {
            const Dimension& dim = *it;
            *it++;
            if (it == _dimensions.end()) {

                getElements(index, result);
                if (index - dim.inc >= 0)
                    getElements(index - dim.inc, result);
                if (index + dim.inc < _tableSize)
                    getElements(index + dim.inc, result);

            } else {
                searchNeighbors(index, it, result);
                if (index - dim.inc >= 0)
                    searchNeighbors(index - dim.inc, it, result);
                if (index + dim.inc < _tableSize)
                    searchNeighbors(index + dim.inc, it, result);
            }
        }

    private:
        typedef std::vector<Cell> Table;
        Table _table;
        int _tableSize;
        std::vector<Dimension> _dimensions;
    };

}}} // end namespaces

#endif // end include guard
