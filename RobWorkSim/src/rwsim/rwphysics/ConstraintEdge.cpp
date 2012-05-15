#include "ConstraintEdge.hpp"

using namespace rwsim::simulator;

ConstraintEdge::ConstraintEdge(
            CNodePair n,
            EdgeType type,
            double touchDist,
            double penDist,
            double sepDist):
    _color(0),
    _nodes(n),
    _type(type),
    _touchDist(touchDist),
    _penDist(penDist),
    _sepDist(sepDist),
    _resting(false)
{
    _state[0] = _state[1] = _state[2] = Static;

    _shortestDist[0] = 0.001;
    _shortestDist[1] = 0.001;

    _nodes.first->addEdge(*this);
    _nodes.second->addEdge(*this);
}

ConstraintEdge::ConstraintEdge(
            CNodePair n,
            EdgeType type, int id):
    _color(0),
    _nodes(n),
    _type(type),
    _touchDist(0),
    _penDist(0),
    _sepDist(0),
    _resting(false),
    _id(id)
{
    _state[0] = _state[1] = _state[2] = Static;

    _shortestDist[0] = 0.001;
    _shortestDist[1] = 0.001;

    _nodes.first->addEdge(*this);
    _nodes.second->addEdge(*this);
}


void ConstraintEdge::saveState(){
    _state[2] = _state[1];
    _state[1] = _state[0];
    _shortestDist[1] = _shortestDist[0];
}

void ConstraintEdge::rollBack(){
    _state[0] = _state[1];
    _state[1] = _state[2];
    _shortestDist[0] = _shortestDist[1];
}

void ConstraintEdge::print(){
    std::cout << "State0 : "; printState(_state[0]);
    std::cout << "State1 : "; printState(_state[1]);
    std::cout << "State2 : "; printState(_state[2]);
    int col = (int)_color;
    std::cout << "Color  : " << col << std::endl;
    std::cout << "Dist   : " << _shortestDist[0] << std::endl;
}

void ConstraintEdge::printState(EdgeState state){
    switch(state){
        case(NewTouch):
            std::cout << "NewTouch";
            break;
        case(PersistentTouch):
            std::cout << "PersistentTouch";
            break;
        case(VanishingTouch):
            std::cout << "VanishingTouch";
            break;
        case(NewProximity):
            std::cout << "NewProximity";
            break;
        case(PersistentProximity):
            std::cout << "PersistentProximity";
            break;
        case(VanishingProximity):
            std::cout << "VanishingProximity";
            break;
        case(Static):
            std::cout << "Static";
            break;
        default:
        break;
     }
     std::cout << std::endl;
}
