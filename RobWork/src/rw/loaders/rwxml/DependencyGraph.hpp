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

#ifndef RW_LOADERS_DEPENDENCYGRAPH_HPP
#define RW_LOADERS_DEPENDENCYGRAPH_HPP

#include <map>

#include <boost/utility.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/visitors.hpp>

namespace rw {
namespace loaders {
/** @addtogroup loaders */
/*@{*/

/**
 * @brief This class is used to create a dependency graph between string nodes
 * and to check if any cycles exist in the dependency graph.
 */
class DependencyGraph
{
public:
    /**
     * @brief Constructor
     */
    DependencyGraph()
    {
    }
    ;

    /**
     * @brief Destructor
     */
    virtual ~DependencyGraph()
    {
    }
    ;

    /**
     * @brief Add dependency from node "fromA" to node "toB"
     * @param fromA [in] name of first node
     * @param toB [in] name of second node
     */
    void addDependency(const std::string& fromA, const std::string& toB);

    /**
     * @brief checks if there are any cycles in the dependency graph
     *
     * @return true if there are any cycles
     */
    bool hasCycleDependency();

private:

    struct cycle_detector: public boost::dfs_visitor<>
    {
        cycle_detector(bool& has_cycle) :
                m_has_cycle(has_cycle)
        {
        }

        template<class Edge, class Graph>
        void back_edge(Edge, Graph&)
        {
            m_has_cycle = true;
        }

    protected:
        bool& m_has_cycle;
    };

private:
    typedef std::pair<std::string, std::string> Edge;
    typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, std::string> DGraph;
    DGraph _g;
    std::map<std::string, int> _map;

};
/*@}*/
}
}
#endif /*RW_LOADERS_DEPENDENCYGRAPH_HPP*/
