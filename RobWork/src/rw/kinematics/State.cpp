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

using namespace rw::common;
using namespace rw::kinematics;

State::State() {}

Frame* State::getFrame(int id){
    return _q_state.getStateSetup()->getFrame(id);
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

    // for each StateData in from.StateSetup copy its Q values to
    // to this.qstate
    BOOST_FOREACH(const boost::shared_ptr<StateData>& f, fromStateDatas){
        // check if frame exist in state
        if( f==NULL )
            continue;
        const StateData& data = *f;
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
    _stateUniqueId = from.getUniqueId();
}


Ptr<StateStructure> State::getStateStructure() const {
    return _tree_state.getStateSetup()->getTree();
}
