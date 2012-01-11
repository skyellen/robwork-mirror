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


#include "DependencyGraph.hpp"

#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/visitors.hpp>


using namespace boost;
using namespace rw::loaders;


void DependencyGraph::addDependency( const std::string& from, const std::string& to )
{
    int fromi,toi;
    std::map<std::string,int>::iterator fromindex = _map.find(from);
    if( fromindex == _map.end() ){
        fromi = _map.size();
        _map[from] = fromi;
    } else {
        fromi = (*fromindex).second;
    }

    std::map<std::string,int>::iterator toindex = _map.find(to);
    if( toindex == _map.end() ){
        toi = _map.size();
        _map[to] = toi;
    } else {
        toi = (*toindex).second;
    }
//    std::cout << "fromi: " << fromi << " toi: "<< toi << std::endl;
    boost::add_edge(fromi, toi, _g);
}

bool DependencyGraph::hasCycleDependency()
{

    bool has_cycle = false;
    cycle_detector vis(has_cycle);
    depth_first_search(_g, visitor(vis));
//    std::cout << "The graph has a cycle? " << has_cycle << std::endl;
    return has_cycle;
}
