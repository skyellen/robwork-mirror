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

#include "State.hpp"

#include "Frame.hpp"
#include "StateSetup.hpp"

#include <boost/foreach.hpp>

using namespace rw::kinematics;

State::State() {}

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
}
