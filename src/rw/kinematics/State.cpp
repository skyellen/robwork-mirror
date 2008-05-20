/*********************************************************************
 * RobWork Version 0.2
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

State::State()
{    
}

void State::copy(const State &newstate){
    // for each StateData in state.StateSetup copy its Q values to
    // to this.qstate

    const QState& qstate = newstate.getQState();
    if(qstate.getStateSetup()==NULL){

        return;
    }
    
    const std::vector<boost::shared_ptr<StateData> >& stateDatas = 
        qstate.getStateSetup()->getStateData();

    BOOST_FOREACH(const boost::shared_ptr<StateData>& f, stateDatas){
        // check if frame exist in state
        if( f==NULL )
            continue;
        const StateData& data = *f; 

        
        int offset = qstate.getStateSetup()->getOffset(data);

        if(offset<0)
            continue;

        const double *vals = qstate.getQ( data );
        _q_state.setQ( data, vals ) ;
    }

    
    // for each DAF in state.StateSetup copy its parent
    // association to this.treestate
    const TreeState& tstate = newstate.getTreeState();
    const std::vector<Frame*>& dafs = qstate.getStateSetup()->getTree()->getDAFs();
    BOOST_FOREACH(Frame* daf, dafs){
        // check if daf is still in newstate
        int dafidx = tstate.getStateSetup()->getDAFIdx(daf);
        if( dafidx<0 )
            continue;
        // also check if the parent that is 
        // currently associated, exist in this state
        Frame *parent = daf->getDafParent(newstate);
        RW_ASSERT(parent); // cannot and must not be null
        int parentIdx = _tree_state.getStateSetup()->getOffset(*parent);
        if( parentIdx<0 )
            continue;
        // now its secure to attach the frame in this state
        _tree_state.attachFrame(daf, parent);
    }
    
}
