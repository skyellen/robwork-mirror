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

#include "StateStructure.hpp"

#include <boost/foreach.hpp>

#include <rw/math/Transform3D.hpp>
#include "FixedFrame.hpp"
#include "State.hpp"
#include "QState.hpp"
#include "StateSetup.hpp"
#include "TreeState.hpp"

using namespace rw::math;
using namespace rw::kinematics;


StateStructure::StateStructure():
    _version(0),
    _root(NULL)
{
    _root = new FixedFrame("WORLD",Transform3D<>::identity());
    // now add the state data of the frame
    //_frames.push_back(_root);
    addDataInternal(_root);
    _frames[_root->getID()] = _root;
    // and setup the static frame-parent relationship
    _root->setParent(NULL);
    _frameIdxMap[_root->getName()] = _root->getID();

    updateDefaultState();
}

StateStructure::~StateStructure()
{
    // for now everything is destructed with shared_ptr
}

bool StateStructure::has(StateData *data){
    const int id = data->getID();
    return (id>=0 && id<getMaxID() && _allDatas[id]!=NULL);
}

void StateStructure::addData(StateData *data){

    addDataInternal(data);

    updateDefaultState();
}

void StateStructure::addDataInternal(StateData *data){
    // frame must not be in tree allready
    RW_ASSERT(!has(data));
    _version++;
    const int id = allocateDataID();
    data->setID(id);
    // There is no turning back. Ownership is forever taken.
    boost::shared_ptr<StateData> sharedData( data );
    _allDatas.at(id) = sharedData;
    _currDatas.at(id) = sharedData;
    // make room in the initial state data array
    // now frame must be in the tree
    RW_ASSERT(has(data));
    // lastly update the default state
}

void StateStructure::addFrame(Frame *frame, Frame *parent){
    // both frame and parent must be well defined
    RW_ASSERT(frame && parent);
    // and parent must exist in the tree, but not the frame
    RW_ASSERT(!has(frame));
    RW_ASSERT(has(parent));
    // and lastly we must check if the frame has been added to other StateStructure
    RW_ASSERT( frame->getID()==-1 );
    // check if frame name is unique
    if(findFrame(frame->getName())!=NULL)
        RW_THROW("Frame name is not unique: "<< frame->getName() );

    // update the parent child relationships
    frame->setParent(parent);
    parent->addChild(frame);
    // now add the state data of the frame
    addDataInternal(frame);
    // add the frame to the framelist
    _frames[frame->getID()] = frame;
    // remember to add the frame to the frameIdxMap
    _frameIdxMap[frame->getName()] = frame->getID();

    updateDefaultState();

    // and in the end cleanup
    cleanup();
}

void StateStructure::addDAF(Frame *frame, Frame *parent){
    // both frame and parent must be well defined
    RW_ASSERT(frame && parent);
    // and parent must exist in the tree, but not the frame
    RW_ASSERT(!has(frame) && has(parent));
    // and lastly we must check if the frame has been added to other StateStructure
    RW_ASSERT( frame->getID()==-1 );
    // check if frame name is unique
    if(findFrame(frame->getName())!=NULL)
        RW_THROW("Frame name is not unique!");

    // now add the state data of the frame
    addDataInternal(frame);
    // push it to the frame list and DAF list
    _DAFs.push_back(frame);
    _frames[frame->getID()] = frame;

    updateDefaultState();

    // and insert the dynamic frame-parent relationship in the default state
    _defaultState.getTreeState().attachFrame(frame,parent);
    // remember to add the frame to the frameIdxMap
    _frameIdxMap[frame->getName()] = frame->getID();
    // and in the end cleanup
    cleanup();
}

void StateStructure::remove(StateData *data){
    RW_ASSERT(data);
    RW_ASSERT(data->getID()>0);
    _version++;
    int id = data->getID();
    if( _frames[id]!=NULL ){
        // check if frame has staticly connected children
        Frame::iterator_pair iter = _frames[id]->getChildren();
        if( iter.first!=iter.second )
            RW_THROW("Frame has staticly connected children and therefore cannot be removed from tree!");
        // remember to remove the from the frameIdxMap
        _frameIdxMap.erase(_frames[id]->getName());
    }

    // setting data in current data list to null
    _currDatas[id] = boost::shared_ptr<StateData>();
    // update default state
    // the dynamicly attached frames will automaticly be attached to world
    updateDefaultState();

    // perform cleanup
    cleanup();
}

State StateStructure::upgradeState(const State& oldState)
{
    State state = _defaultState;
    state.copy( oldState );
    return state;
}

State StateStructure::getDefaultState()
{
    return _defaultState;
}

void StateStructure::setDefaultState(const State &state)
{
    // check if state version
    _defaultState.copy(state);
}

void StateStructure::cleanup(){
    // first run through StateSetup list and remove all state setups that are not
    // used anymore
    StateSetupList::iterator iter = _setups.begin();
    while(iter!=_setups.end()){
        if((*iter).use_count()>1){
            ++iter;
        } else {
            //std::cout << "Erasing StateSetup v." << (*iter)->getVersion() << std::endl;
            // erase the setup from the list
            iter = _setups.erase(iter);
        }
    }

    // NEXT run through statedata list and remove all data that
    // is not pointed to by anything else.
    // remember to also remove instances from the daf and frame list
    BOOST_FOREACH(boost::shared_ptr<StateData>& data, _allDatas){
        if(data!=NULL && data.use_count()==1){
            // delete the data and put it on available list
            const int id = data->getID();
            // test if data is a daf
            for(size_t i=0;i<_DAFs.size();i++){
                if(_DAFs[i]!=NULL && _DAFs[i]->getID()==id){
                    _DAFs[i] = NULL;
                    break;
                }
            }
            // if its a frame then remove it from its parent
            if (_frames[ id ] != NULL ){
                Frame *frame = _frames [id];
                if(frame->getParent()!=NULL){
                    frame->getParent()->removeChild( frame );
                }
                _frames[id] = NULL;
            }

            data = boost::shared_ptr<StateData>();
            _availableDataIds.push_back(id);

        }
    }


}

void StateStructure::updateDefaultState(){
    // first create a StateSetup
    boost::shared_ptr<StateSetup> setup(new StateSetup(_version,*this, _currDatas));
    _setups.push_back(setup);
    // construct qstate and tree state and then a new State
    QState qstate(setup);
    TreeState tstate(setup);
    State newState( qstate , tstate );
    // copy the default values into newState
    newState.copy( _defaultState );
    _defaultState = newState;
}

int StateStructure::allocateDataID()
{
    // check if any ids are currently available
    if (_availableDataIds.empty()) {
        // if not then generate a new id
        const int id = _allDatas.size();
        _allDatas.push_back(boost::shared_ptr<StateData>());
        _currDatas.push_back(boost::shared_ptr<StateData>());
        _frames.push_back(NULL);
        RW_ASSERT(_allDatas.size() == _currDatas.size());
        return id;
    } else {
        const int id = _availableDataIds.back();
        _availableDataIds.pop_back();
        return id;
    }
}

Frame* StateStructure::findFrame(const std::string& name) const {
    typedef FrameIdxMap::const_iterator I;
    const I pos = _frameIdxMap.find(name);
    if (pos == _frameIdxMap.end())
        return NULL;
    int idx = pos->second;
    if( _allDatas[idx]==NULL )
        return NULL;
    return _frames[pos->second];
}
