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


#include "State.hpp"

#include "Frame.hpp"
#include "StateSetup.hpp"

#include <boost/foreach.hpp>

//#include <rw/common/InputArchive.hpp>
//#include <rw/common/OutputArchive.hpp>

using namespace rw::common;
using namespace rw::kinematics;

State::State() { }

State::State(const QState& q_state,
      const TreeState& tree_state,
      int stateUniqueId) :
    _tree_state(tree_state),
    _q_state(q_state),
    _stateUniqueId(stateUniqueId)
{
    _cache_state.resize( _tree_state.getStateSetup()->getMaxCacheIdx() );
}


State::~State(){ }

Frame* State::getFrame(int id){
    return _q_state.getStateSetup()->getFrame(id);
}

rw::common::Ptr<StateCache> State::getCache(int id){
    int cacheIdx = _q_state.getStateSetup()->getCacheIdx(id);
    if(cacheIdx<0)
        return NULL;
    return _cache_state[cacheIdx];
}

rw::common::Ptr<StateCache> State::getCache(int id) const {
    int cacheIdx = _q_state.getStateSetup()->getCacheIdx(id);
    if(cacheIdx<0)
        return NULL;
    return _cache_state[cacheIdx];
}

void State::setCache(int id, rw::common::Ptr<StateCache> cache){
    int cacheIdx = _q_state.getStateSetup()->getCacheIdx(id);
    if(cacheIdx<0)
        return;
    _cache_state[cacheIdx] = cache;
}

void State::copy(const State &from){
    // make sure the state too be copied is a valid state
    const QState& fromQState = from.getQState();
    if(fromQState.getStateSetup()==NULL){
        return;
    }

    // get state data from the from state
    const std::vector<boost::shared_ptr<StateData> >& fromStateDatas =
        fromQState.getStateSetup()->getStateData();

    const std::vector<boost::shared_ptr<StateData> >& toStateDatas =
    		getQState().getStateSetup()->getStateData();

    // for each StateData in from.StateSetup copy its Q values to
    // to this.qstate
    for(size_t i=0; i<fromStateDatas.size(); i++){
    //BOOST_FOREACH(const boost::shared_ptr<StateData>& f, fromStateDatas){
    	const boost::shared_ptr<StateData>& f = fromStateDatas[i];
        // check if frame exist in state
        if( f==NULL )
            continue;
        // make sure the statedata is also available in this qstate
        if(i>=toStateDatas.size() || toStateDatas[i]==NULL)
        	continue;

        const StateData& data = *f;

        if( data.hasCache() ){
            int fromCacheIdx = fromQState.getStateSetup()->getCacheIdx(data);
            int toCacheIdx = _q_state.getStateSetup()->getCacheIdx(data);
            if(fromCacheIdx>=0 && toCacheIdx>=0){
                _cache_state[toCacheIdx] = from._cache_state[fromCacheIdx];
            }
        }

        int offset = fromQState.getStateSetup()->getOffset(data);
        if(offset<0)
            continue;

        const double *vals = fromQState.getQ( data );
        _q_state.setQ( data, vals ) ;
    }

    // for each DAF in state.StateSetup copy its parent
    // association to this.treestate
    const TreeState& tstate = from.getTreeState();
    const std::vector<Frame*>& dafs = fromQState.getStateSetup()->getTree()->getDAFs();
    BOOST_FOREACH(Frame* daf, dafs){


        // check if daf is still in newstate
        int dafidx = tstate.getStateSetup()->getDAFIdx(daf);
        if( dafidx<0 )
            continue;

        // also check if the parent that is
        // currently associated, exist in this state
        Frame *parent = daf->getDafParent(from);
        RW_ASSERT(parent); // cannot and must not be null
        int parentIdx = _tree_state.getStateSetup()->getOffset(*parent);
        if( parentIdx<0 )
            continue;

        // now its secure to attach the frame in this state
        _tree_state.attachFrame(daf, parent);
    }
    // the state id is unique in regard to the static content, eg. adding a new frame would change the id
    // so only upgrade should change the id
    //_stateUniqueId = from.getUniqueId();
}

State State::clone(){
    State state;
    state.clone(*this);
    return state;
}

/**
 * @brief performs a deep copy of \b src into this state.
 * @param state [in] the state that is to be cloned
 */
void State::clone( const State& from ){
    // first copy the qstate and tree state, and shallow copy cache
    *this = from;
    // next run through cach and perform deep copies
    BOOST_FOREACH(StateCache::Ptr &cache, _cache_state){
        if(cache!=NULL)
            cache = cache->clone();
    }
}



Ptr<StateStructure> State::getStateStructure() const {
    return _tree_state.getStateSetup()->getTree();
}


void State::upgrade(){
    // we only upgrade if the version differs
    if(_tree_state.getStateSetup()->getTree()->getDefaultState().getUniqueId() != _stateUniqueId ){
        upgradeTo(_tree_state.getStateSetup()->getTree()->getDefaultState());
    }
}

const State& State::getDefault( StateData* data ){
	return data->getStateStructure()->getDefaultState();
}

/*
void State::add(Stateless& obj){
	obj.registerIn( getStateStructure() );
}
*/

void State::read(rw::common::InputArchive& iarchive, const std::string& id){
	// read the state setup
/*
	std::vector<boost::uint8_t> varray;
	iarchive.read(varray, "QStateArray");
	//_q_state = QState( rw::math::Q( (double*)&varray(0), varray.size()/sizeof(double) ) );

	// now to the tree state

*/
}

void State::write(rw::common::OutputArchive& oarchive, const std::string& id) const{
	// the id of the state
/*
	// add id
	oarchive.write( _stateUniqueId, "StateID");

	// add the Q state
    boost::uint8_t* array = (uint8_t)&_q_state(0);
    unsigned int array_byte_size = _q_state.size()*sizeof(double);

    std::vector<boost::uint8_t> varray(array,array+array_byte_size);
    oarchive.write( varray, "QStateArray");

    // now read the tree state
    putTreeState(state, frames);
*/
}



