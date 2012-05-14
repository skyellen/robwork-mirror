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

    //*********'*** Next create setup data for the StateCache data
    _sdataTCacheIdx.resize( tree.getMaxID() );
    // initialize the _offsets with -1 such that empty data's return -1 when requested
    BOOST_FOREACH(int &offsetIdx, _sdataTCacheIdx){ offsetIdx = -1; }
    // Traverse the data and calculate the offsets.
    offset = 0;
    BOOST_FOREACH(boost::shared_ptr<StateData>& dval, _datas){
        if( dval==NULL )
            continue;
        if(dval->hasCache()){
            _sdataTCacheIdx.at(dval->getID()) = offset;
            offset++;
        }
    }
    _nrCaches = offset;
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
