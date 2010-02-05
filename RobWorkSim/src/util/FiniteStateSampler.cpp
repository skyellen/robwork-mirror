#include "FiniteStateSampler.hpp"

using namespace rw::kinematics;

FiniteStateSampler::FiniteStateSampler(const State& state, int n):
    _type(FiniteStateSampler::SINGLE_STATE),
    _n(n),
    _states(1,state),
    _empty( n>0 )
{}

FiniteStateSampler::FiniteStateSampler(const std::vector<State>& states, int n):
    _type(FiniteStateSampler::MULTIPLE_STATE),
    _n(n),
    _states(states),
    _empty( n>0 && states.size()>0)
{
}

FiniteStateSampler::~FiniteStateSampler(){}


bool FiniteStateSampler::sample(rw::kinematics::State& state){
    if(_n==0){
        _empty = true;
        return false;
    }

    switch(_type){
    case(SINGLE_STATE):
        state = _states[0];
        _n--;
        break;
    case(MULTIPLE_STATE):
        state = _states[_cidx];
        _cidx++;
        if(_states.size()==_cidx){
            _cidx=0;
            _n--;
        }
        break;
    default: RW_THROW("UNSUPPORTED TYPE");
    }
    if( _n==0 )
        _empty=true;

    return true;
}

bool FiniteStateSampler::empty() const{
    return _empty;
}

