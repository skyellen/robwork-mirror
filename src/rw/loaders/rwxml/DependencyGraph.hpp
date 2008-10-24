/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RW_LOADERS_DEPENDENCYGRAPH_HPP
#define RW_LOADERS_DEPENDENCYGRAPH_HPP

#include <iostream>
#include <iterator>
#include <algorithm>
#include <time.h>
#include <map>

#include <boost/utility.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/visitors.hpp>

namespace rw { namespace loaders {
	/** @addtogroup loaders */
    /*@{*/

    /**
     * @brief This class is used to create a dependency graph between string nodes
     * and to check if any cycles exist in the dependency graph.
     */
    class DependencyGraph
    {
    public:
        /*
         * @brief Constructor
         */
        DependencyGraph(){};

        /*
         * @brief Destructor
         */
        virtual ~DependencyGraph(){};

        /*
         * @brief Add dependency from node "fromA" to node "toB"
         * @param fromA [in] name of first node
         * @param fromB [in] name of second node
         */
        void addDependency(const std::string& fromA, const std::string& toB );

        /*
         * @brief checks if there are any cycles in the dependency graph
         *
         * @return true if there are any cycles
         */
        bool hasCycleDependency();

    private:

        typedef std::pair<std::string,std::string> Edge;
        typedef boost::adjacency_list<boost::listS, boost::vecS,boost::directedS,
                                      std::string
                                      > DGraph;
        DGraph _g;
        std::map<std::string,int> _map;
    };
    /*@}*/
}}
#endif /*RW_LOADERS_DEPENDENCYGRAPH_HPP*/
