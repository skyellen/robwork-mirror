#include "FiniteStateSampler.hpp"

#include <boost/foreach.hpp>

#include <rw/kinematics/State.hpp>
#include <rw/math/Random.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::util;

FiniteStateSampler::FiniteStateSampler(
		const State& state, int n, FiniteStateSampler::SamplerType type):
    _type(type),
    _n(n), _cidx(0),
    _states(1,state),
    _empty( n>0 )
{}

FiniteStateSampler::FiniteStateSampler(
		const std::vector<State>& states, int n, FiniteStateSampler::SamplerType type):
    _type(type),
    _n(n),_cidx(0),
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

    if(_states.size()==1){
        state = _states[0];
        _n--;
    } else {
		switch(_type){
			case(ORDERED_SAMPLING):{
				state = _states[_cidx];
				_n--;
				if(_states.size()==(size_t)_cidx){
					_cidx=0;
				}
				break;
			}
			case(RANDOM_SAMPLING):{
				int idx = Random::ranI(0,(int)_states.size());
				state = _states[idx];
				_n--;
				break;
			}
			default: RW_THROW("UNSUPPORTED TYPE");
		}
    }
    if( _n==0 )
        _empty=true;
    if( _n<0 )
    	_n=-1;
    return true;
}

bool FiniteStateSampler::empty() const{
    return _empty;
}

void FiniteStateSampler::addState(const rw::kinematics::State& state){
	_states.push_back(state);
}

void FiniteStateSampler::addStates(const std::vector<rw::kinematics::State>& states){
	BOOST_FOREACH(const State& state, states){
		_states.push_back(state);
	}
}

void FiniteStateSampler::setStates(const std::vector<rw::kinematics::State>& states){
	_states = states;
}


