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


#include "TreeState.hpp"

#include "Frame.hpp"
#include "StateSetup.hpp"

#include <rw/common/StringUtil.hpp>

using namespace rw::kinematics;
using namespace rw::common;

namespace {
    int nrOfDafs(boost::shared_ptr<StateSetup> setup){
        return setup->getMaxDAFIdx();
    }

    int nrOfIDs(boost::shared_ptr<StateSetup> setup){
        return setup->getMaxChildListIdx();
    }

    Frame* getRoot(boost::shared_ptr<StateSetup> setup){
        return  setup->getTree()->getRoot();
    }

    int getRootIdx(boost::shared_ptr<StateSetup> setup){
        return getRoot(setup)->getID();
    }

    // a global empty framelist
    const std::vector<Frame*> emptyFrameList(0);
}

TreeState::TreeState()
{}

TreeState::~TreeState()
{}

TreeState::TreeState(const TreeState &src){
    _setup = src._setup;

    _parentIdxToChildList = src._parentIdxToChildList;

    _childLists = src._childLists;

    _dafIdxToParentIdx = src._dafIdxToParentIdx;
}

boost::shared_ptr<StateSetup> TreeState::getStateSetup() const{
    return _setup;
}

TreeState::TreeState(boost::shared_ptr<StateSetup> setup):
    _setup(setup),
    _parentIdxToChildList( nrOfIDs(setup), -1 ),
    _childLists(1, FrameList(nrOfDafs(setup))),
    _dafIdxToParentIdx( nrOfDafs(setup), getRootIdx(setup) )
{
    // initialize child list such that
    const std::vector<Frame*> dafs = setup->getDafs();
    for(int i=0;i<nrOfDafs(setup); i++){
        _childLists[0].at(i) = dafs[i];
    }
    // remember to point the root frame toward its children
    int rootIdx = setup->getChildListIdx(getRoot(setup));
    _parentIdxToChildList[rootIdx] = 0;

}

const Frame* TreeState::getParent(const Frame* frame) const
{
    // first get the DAF idx
    int dafidx = _setup->getDAFIdx(frame);
    // if -1 then frame is not a DAF
    if( dafidx == -1 ) return NULL;
    // next use the idx to get the real frame idx
    int idx = _dafIdxToParentIdx[dafidx];
    // if -1 then DAF has no parent
    if( idx == -1 ) return NULL;
    return _setup->getFrame(idx);
}

Frame* TreeState::getParent(Frame* frame) const
{

    // first get the DAF idx
    int dafidx = _setup->getDAFIdx(frame);
    // if -1 then frame is not a DAF
    if( dafidx == -1 )
        return NULL;

    // next use the idx to get the real frame idx
    int idx = _dafIdxToParentIdx[dafidx];
    // if -1 then DAF has no parent
    if( idx == -1 )
        return NULL;

    return _setup->getFrame(idx);
}

const TreeState::FrameList& TreeState::getChildren(const Frame* frame) const
{
    // first get the idx that maps into our Childrenidx-list map
    const int idx = _setup->getChildListIdx(frame);
    // if -1 then frame has no DAF children
    if( idx == -1 )
        return emptyFrameList;

    // next use the idx to get the real frame idx
    const int childlistidx = _parentIdxToChildList[idx];
    if( childlistidx == -1 )
        return emptyFrameList;

    return _childLists[childlistidx];
}

void TreeState::attachFrame(Frame* frame, Frame* parent)
{
    // If it is not a DAF:
    Frame* static_parent = frame->getParent();
    if (static_parent) {
        RW_THROW(
            "Can't attach frame "
            << StringUtil::quote(frame->getName())
            << " to frame "
            << StringUtil::quote(parent->getName())
            << ".\n"
            << "The frame is not a DAF but is statically attached to frame "
            << StringUtil::quote(static_parent->getName()));
    }

    if (frame == parent)
        RW_THROW(
            "Can't attach frame "
            << StringUtil::quote(frame->getName())
            << " to itself.");

    //if ( !has(frame) || !has(parent) )
    //    RW_THROW("Frame is not valid in this tree state!");

    // first get the DAF idx
    int dafidx = _setup->getDAFIdx(frame);
    RW_ASSERT( dafidx>=0 ); // should allways be a daf since we allready checked that

    // check if daf allready has a parent
    Frame *p = getParent(frame);
    if( p != NULL ){
        if( p==parent) return;// then we are done
        //else remove child from ps list
        int idx = _setup->getChildListIdx(p);

        const int childlistidx = _parentIdxToChildList[idx];

        RW_ASSERT(childlistidx!=-1);

        FrameList *childrenList = &_childLists[childlistidx];
        FrameList::iterator iter = childrenList->begin();
        for(;iter!=childrenList->end();++iter){
            if( *iter == frame ) {
                childrenList->erase(iter);
                break;
            }
        }
    }

    // next get the idx that maps into our Childrenidx-list map
    int idx = _setup->getChildListIdx(parent);
    if(idx==-1){
        RW_THROW("Tried to attach daf to a frame that is invalid in the current state!");
    }

    // get the children vector if any else create one
    //FrameList *childrenList = _parentIdxToChildList[idx];
    int childlistidx = _parentIdxToChildList[idx];
    if( childlistidx==-1 ){
        childlistidx = (int)_childLists.size();
        _childLists.push_back(FrameList(1,frame));
        _parentIdxToChildList[idx] = childlistidx;
    } else {
        _childLists[childlistidx].push_back(frame);
    }
    // lastly remember to update the _dafToParent map
    _dafIdxToParentIdx[dafidx] = parent->getID();
}
