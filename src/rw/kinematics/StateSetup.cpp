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

#include "StateSetup.hpp"

#include <boost/foreach.hpp>

using namespace rw::kinematics;

StateSetup::StateSetup(int version, StateStructure& tree,
                       const std::vector<boost::shared_ptr<StateData> >& stateDatas):
            _version(version),_tree(&tree),_datas(stateDatas),
            _initMaxID(tree.getMaxID())
{
    //*********'*** first create setup data for the QState
    _offsets.resize( tree.getMaxID() );
    // initialize the _offsets with -1 such that empty data's return -1 when requested
    BOOST_FOREACH(int &offsetIdx, _offsets){ offsetIdx = -1; }
    // Traverse the data and calculate the offsets.
    int offset = 0;
    BOOST_FOREACH(boost::shared_ptr<StateData>& dval, _datas){
        if( dval==NULL )
            continue;
        _offsets.at(dval->getID()) = offset;
        offset += dval->size();
    }
    _dof = offset;

    //************* next create setup data for the TreeState
    // create daf cildren offsets
    // create parent-children offsets
    _dafidx.resize( tree.getMaxID() );
    _dafChildidx.resize( tree.getMaxID() );

    // initialize the _offsets with -1 such that empty data's return -1 when requested
    BOOST_FOREACH(int &idx, _dafChildidx){ idx = -1; }

    // iterate over the frames and register valid frames
    int nrOfValidFrames = 0;
    const std::vector<Frame*>& frames = _tree->getFrames();
    BOOST_FOREACH(const Frame *frame, frames){
        if( frame==NULL )
            continue;
        if( _datas[frame->getID()]==NULL )
            continue;
        _dafChildidx.at(frame->getID()) = nrOfValidFrames;
        nrOfValidFrames++;
    }

    BOOST_FOREACH(int &idx, _dafidx) { idx = -1; }
    int nrOfDAF = 0;
    const std::vector<Frame*>& dafs = _tree->getDAFs();
    BOOST_FOREACH(Frame *daf, dafs){
        if( daf==NULL )
            continue;
        if( _datas[daf->getID()]==NULL )
            continue;
        _dafidx.at(daf->getID()) = nrOfDAF;
        _dafs.push_back(daf);
        nrOfDAF++;
    }

    _nrOfDAF = nrOfDAF;
    _nrOfValidFrames = nrOfValidFrames;
}
