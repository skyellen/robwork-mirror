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


#ifndef RW_KINEMATICS_STATESTRUCTURE_HPP
#define RW_KINEMATICS_STATESTRUCTURE_HPP

/**
   @file StateStructure.hpp
*/

#include "Frame.hpp"
#include <boost/shared_ptr.hpp>
#include <vector>
#include "State.hpp"

#include <rw/common/Ptr.hpp>

namespace rw { namespace kinematics {
    class StateSetup;

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief the StateStructure is responsible for handling a
     * structure of StateData and Frames
     */
    class StateStructure {
    public:

        /**
         * @brief constructs a frame tree with a default root frame
         * with the name "WORLD".
         */
        StateStructure();

        /**
         * @brief destructor
         */
        virtual ~StateStructure();

        /**
         * @brief tests if StateData data exist in this StateStructure
         *
         * @return true if the data was found, false otherwise
         *
         * @note the search includes the union of StateData in all
         * StateSetup's that belong to the StateStructure
         */
        bool has(StateData *data);

        /**
         * @brief gets the max ID of any StateData/Frame currently in the tree.
         *
         * All frame/data IDs (see StateData::getID()) for the data of the tree are
         * lower than this number (and greater than or equal to zero).
         *
         */
        int getMaxID() const { return _allDatas.size(); }

        /**
         * @brief adds a statedata to the frame tree and allocates memory
         * for its states. This method updates the default
         * state.
         *
         * @note Ownership is taken, the data object may not have been added to
         * any StateStructure before.
         */
        void addData(StateData *data);

        /**
         * @brief adds a frame to the frame tree and staticly associates
         * the frame with the parent frame. This method updates the default
         * state.
         *
         * @note the parent frame must exist in the frame tree and cannot be
         * NULL.
         */
        void addFrame(Frame *frame, Frame *parent);

        /**
         * @brief adds a DAF to the frame tree and dynamicly associates
         * the frame with a parent frame.
         *
         * @note the parent frame must exist in the frame tree and cannot be
         * NULL.
         */
        void addDAF(Frame *frame, Frame *parent);

        /**
         * @brief removes a StateData object from the tree. The actual
         * deletion of the object will happen when no States depend on
         * the StateData anymore.
         * @param data [in] pointer to object that is to be removed
         *
         * @note if the data object is a frame and it has staticly connected
         * children then the remove operation is illigal.
         *
         * @note if the data object is a frame and it has dynamicly attached
         * children then all of these will change parent relation ship such that
         * world will become their parent.
         */
        void remove(StateData *data);

        /**
         * @brief upgrades the state to the default state, but without
         * clearing the values of the state.
         * @param oldState [in] the state that should be upgraded
         * @return the upgraded state
         */
        State upgradeState(const State& oldState);

        /**
         * @brief get the default state of the frame tree
         * @return the default tree state
         */
        State getDefaultState();

        /**
         * @brief set the default state of the dynamic frame tree
         * if the given state is an older state then states valid in both
         * new and old version will be copied to the default state.
         */
        void setDefaultState(const State &state);

        /**
         * @brief All state data in the tree.
         * @return All state data in the tree
         */
        const std::vector<boost::shared_ptr<StateData> >& getStateData() const{
            return _allDatas;
        }

        /**
         * @brief All frames of the tree. Notice that elements in
         * this vector can be NULL
         *
         * @return All frames of the tree.
         */
        const std::vector<Frame*>& getFrames() const{
            return _frames;
        }

        /**
         * @brief All DAFs of the tree.
         *
         * @return All DAFs of the tree.
         */
        const std::vector<Frame*>& getDAFs() const{
            return _DAFs;
        }

        /**
         * @brief get root of state structure
         * @return the root frame of the StateStructure
         */
        const Frame* getRoot() const {return _root;}

        /**
         * @brief get root of state structure
         * @return the root frame of the StateStructure
         */
        Frame* getRoot(){return _root;}

        /**
         * @brief destructs all frames and statedata that is not used any more.
         */
        void cleanup();

        /**
         * @brief Returns frame with the specified name.
         *
         * If multiple frames has the same name, the first frame encountered
         * will be returned. If no frame is found, the method returns NULL.
         *
         * @param name [in] name of Frame.
         *
         * @return The frame with name \b name or NULL if no such frame.
         */
        kinematics::Frame* findFrame(const std::string& name) const;

    private:

        void updateDefaultState();

        int allocateDataID();

        void addDataInternal(StateData *data);

    private:
        // this specify the version of the initial/default data/setup
        int _version;

        // this specify the root frame
        Frame *_root;

        // the default state
        State _defaultState;

        //********** stuff for creating the default state
        // daf parent intial state
        std::vector<int> _initialDafParents;

        // the public setup history, when all references to one setup
        // is gone it should be removed from the list
        typedef std::vector<boost::shared_ptr<StateSetup> > StateSetupList;
        StateSetupList _setups;

        // the complete list of frames
        std::vector<Frame*> _frames;

        // the list of all dynamic attachable frames
        std::vector<Frame*> _DAFs;

        // the complete list of statedata, this define the IDs
        std::vector<boost::shared_ptr<StateData> > _allDatas;
        std::vector<boost::shared_ptr<StateData> > _currDatas;

        // list for keeping track of available ids
        std::vector<int> _availableDataIds;

        // map from string id to frame name
        typedef std::map<std::string, int> FrameIdxMap;
        FrameIdxMap _frameIdxMap;
    };

    typedef rw::common::Ptr<StateStructure> StateStructurePtr;

    /*@}*/
}}

#endif /* RW_KINEMATICS_STATESTRUCTURE_HPP */
