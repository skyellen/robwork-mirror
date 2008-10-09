#ifndef RWLIBS_PATHPLANNERS_PRM_PARTIALINDEXTABLE_HPP
#define RWLIBS_PATHPLANNERS_PRM_PARTIALINDEXTABLE_HPP

#include <rw/math/Q.hpp>
#include <list>
#include <vector>
#include <queue>
#include <iostream>
#include <boost/function.hpp>

#include <rw/common/macros.hpp>


namespace rwlibs {
namespace pathplanners {
namespace prm {

/**
 * @brief Provides an Partial Index Table to be used for nearest neighbor search.
 *
 * This class is implemented as a helper for the PRMPlanner.
 */
template<class T>
class PartialIndexTable {
private:

    struct Dimension {
    public:
        int index;
        int length;
        double qoffset;
        double stepsize;
        int offset;
        int inc;
    };

    /**
     * @brief Compares which of two dimensions is the best for the table
     */
    friend bool operator<(const Dimension&a, const Dimension &b)
    {
      return a.length < b.length;
    }

    std::vector<Dimension > _dimensions;

    /**
     * @brief Calculates the index for a given configuration
     */
    int getIndex(const rw::math::Q& q) {
        size_t index = 0;
        for (typename std::vector<Dimension>::iterator it = _dimensions.begin(); it != _dimensions.end(); ++it) {
            Dimension dim = *it;
            size_t i1 = static_cast<size_t>((q(dim.index)-dim.qoffset)/dim.stepsize);
            index += i1*dim.inc;
        }
        return index;
    }

    /**
     * @brief Recursive method running through all the dimensions
     */
    void searchNeighbors(int index, typename std::vector<Dimension>::iterator it, std::list<T>& result) {
        const Dimension& dim = *it;
        *it++;
        if (it == _dimensions.end()) {
            result.insert(result.begin(), _table[index].begin(), _table[index].end());
            if (index - dim.inc >= 0)
                result.insert(result.begin(), _table[index - dim.inc].begin(), _table[index - dim.inc].end());
            if (index + dim.inc < _tableSize)
                result.insert(result.begin(), _table[index + dim.inc].begin(), _table[index + dim.inc].end());
        } else {
            searchNeighbors(index, it, result);
            if (index - dim.inc >= 0)
                searchNeighbors(index - dim.inc, it, result);
            if (index + dim.inc < _tableSize)
                searchNeighbors(index + dim.inc, it, result);
        }
    }


public:
    /**
     * @brief Constructs Partial Index Table
     *
     * @param bounds [in] Bounds of the space to partially index
     * @param weights [in] The weights for the WeightedEuclideanMetric used determining whether two nodes are neighbors
     * @param r [in] The distance where two nodes are considered neighbors
     * @param dims [in] The number of dimensions of the table. \b dims has to be within [1, dof]
     */
    PartialIndexTable(const std::pair<rw::math::Q, rw::math::Q>& bounds, rw::math::Q& weights, double r, size_t dims) {
        RW_ASSERT(dims > 0);
        RW_ASSERT(dims <= bounds.first.size());
        std::priority_queue<Dimension> queue;
        for (size_t i = 0; i<bounds.first.size(); i++) {
            size_t divs = (int)std::ceil((bounds.second(i)-bounds.first(i))*weights(i) / r);
            double stepsize = (bounds.second(i)-bounds.first(i)) / divs;
            Dimension dim = {i, divs, bounds.first(i), stepsize, 0, 0};
            queue.push(dim);
        }

        _tableSize = 0;
        size_t n = std::min(dims, queue.size());
        for (size_t i = 0; i<n; i++) {
            Dimension dim = queue.top();
            queue.pop();
            dim.offset = _tableSize;
            dim.inc = std::max((int)_tableSize, 1);
            _tableSize = dim.inc*dim.length;
            _dimensions.push_back(dim);
        }
        _table = new std::list<T>[_tableSize];

    }

    /**
     * @brief Destructor
     */
    ~PartialIndexTable() {
        delete[] _table;
    }

    /**
     * @brief Adds a node to the table
     * @param node [in] Node to add
     * @param q [in] Configuration of node
     */
    void addNode(T& node, const rw::math::Q& q) {
        size_t index = getIndex(q);
        std::cout<<"index = "<<index<<std::endl;
        _table[index].push_back(node);
    }

    /**
     * @brief Removes node from the table
     * @param node [in] Node to remove
     * @param q [in] Configuration associated with the node. Used to find the table entry containing the node.
     */
    void removeNode(T& node, const rw::math::Q& q) {
        size_t index = getIndex(q);
        _table[index].remove(node);
    }

    /**
     * @brief Searches for all potential neighbors.
     *
     * The potential neighbor is those within the cell associated with \b q and all neighboring cells.
     * To find the true neighbors one has to run through the content of the list and make an
     * exact match.
     *
     * @param q [in] Configuration to search neighbors for
     */
    std::list<T> searchNeighbors(rw::math::Q& q/*, boost::function<void(T&, T&, double)> function*/) {
        int index = getIndex(q);
        std::list<T> result;
        searchNeighbors(index, _dimensions.begin(), result);
        return result;
    }

private:
    std::list<T>* _table;
    int _tableSize;

    size_t st1, st2, st3;



};

} //end namespace prm
} //end namespace pathplanners
} //end namespace rwlibs


#endif /*RWLIBS_PATHPLANNERS_PRM_PARTIALINDEXTABLE*/
