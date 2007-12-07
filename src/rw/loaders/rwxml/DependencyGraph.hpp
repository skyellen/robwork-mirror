#ifndef DEPENDENCYGRAPH_HPP_
#define DEPENDENCYGRAPH_HPP_

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
#endif /*DEPENDENCYGRAPH_HPP_*/
