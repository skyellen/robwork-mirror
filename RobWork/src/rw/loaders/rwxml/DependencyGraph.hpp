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

#include <iostream>
#include <iterator>
#include <algorithm>
#include <time.h>
#include <map>
#include <limits.h>

#include <boost/utility.hpp>
#include <boost/graph/adjacency_list.hpp>

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
        /**
         * @brief Constructor
         */
        DependencyGraph(){};

        /**
         * @brief Destructor
         */
        virtual ~DependencyGraph(){};

        /**
         * @brief Add dependency from node "fromA" to node "toB"
         * @param fromA [in] name of first node
         * @param toB [in] name of second node
         */
        void addDependency(const std::string& fromA, const std::string& toB );

        /**
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
