#include "DependencyGraph.hpp"

using namespace boost;
using namespace rw::loaders;

namespace {

    struct print_visitor : public bfs_visitor<> {
        template <class Vertex, class Graph>
        void discover_vertex(Vertex v, Graph&) {
            //std::cout << name[v] << " ";
        }
    };

    struct cycle_detector : public dfs_visitor<>
    {
        cycle_detector(bool& has_cycle)
            : m_has_cycle(has_cycle) { }

        template <class Edge, class Graph>
        void back_edge(Edge, Graph&) {
            m_has_cycle = true;
        }

    protected:
        bool& m_has_cycle;
    };
}

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
