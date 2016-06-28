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


#ifndef RW_KINEMATICS_STATESETUP_HPP
#define RW_KINEMATICS_STATESETUP_HPP

/**
 * @file StateSetup.hpp
 */

#include "Frame.hpp"
#include "StateStructure.hpp"
#include "StateData.hpp"
#include <boost/shared_ptr.hpp>

namespace rw { namespace kinematics {

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief Utility class to help construct a State
     *
     * StateSetup contains the data to share among QState objects and
     * TreeState objects, namely the assignment of offsets to frames,
     * the mapping of frame indexes to indexes in the QState,
     * the mapping of frame indexes to daf and dafparent index in
     * the TreeState,
     */
    class StateSetup
    {
    public:
        /**
         * @brief Creates an empty StateSetup
         */
        StateSetup():
            _version(-1), _tree(NULL),
            _dof(0),_nrOfDAF(0),_nrOfValidFrames(0),
            _initMaxID(0),_nrCaches(0)
        {
        }

        /**
         * @brief Creates a StateSetup from a StateStructure and a number of
         * valid statedata.
         * @param version [in] the version of the StateSetup
         * @param tree [in]
         * @param stateDatas [in] a list of valid statedatas for this version
         */
        explicit StateSetup(int version,
                   StateStructure& tree,
                   const std::vector<boost::shared_ptr<StateData> >& stateDatas);

        //! @brief destructor
        ~StateSetup() {}

        /**
         * @brief The position in QState at which the configuration for \b frame
         * is stored.
         */
        inline int getOffset(const StateData& data) const
        {
            const int id =  data.getID();
            if(id<0 || id>=_initMaxID)
                return -1;
            return _offsets[id];
        }

        inline int getCacheIdx(const StateData& data) const
        {
            const int id =  data.getID();
            if(id<0 || id>=_initMaxID)
                return -1;
            return _sdataTCacheIdx[id];
        }

        /**
         * @brief The total number of doubles allocated by all valid
         * state data in the StateSetup.
         * @return the total number of allocated doubles
         * @note This number equals the length of the QState array.
         */
        inline int size() const { return _dof; }

        /**
         * @brief gets the version of the StateSetup
         * @return the version of the state setup
         */
        inline int getVersion() const { return _version; }

        /**
         * @brief gets the frame with index idx
         * @param id [in] the unique id of the frame
         * @return the frame with id id, else NULL
         */
        inline const Frame* getFrame(int id) const {
            return _tree->getFrames()[id];
        }

        /**
         * @brief gets the frame with index idx
         * @param id [in] the unique id of the frame
         * @return the frame with id id, else NULL
         */
        inline Frame* getFrame(int id) {
            return _tree->getFrames()[id];
        }

        /**
         * @brief gets the index that maps a frame parent into
         * all its daf children.
         * @param parent [in] the parent to the children list
         * @return index into the childlist array in tree state
         */
        int getChildListIdx( const Frame *parent ) const{
            const int id =  parent->getID();
            if(id<0 || id>=_initMaxID)
                return -1;
            return _dafChildidx[parent->getID()];
        }

        /**
         * @brief gets the number of valid frames in the state setup
         */
        int getMaxChildListIdx() const {
            return _nrOfValidFrames;
        }

        /**
         * @brief gets the list of DAFs that are valid in this state setup
         * @return list of DAFs
         */
        const std::vector<Frame*>& getDafs() const{
            return _dafs;
        }

        /**
         * @brief gets the index that maps a DAF into its
         * position in the TreeState daf list
         * @param daf [in] the daf frame
         * @return index into the TreeState daf list
         */
        int getDAFIdx( const Frame *daf ) const {
            const int id =  daf->getID();
            if(id<0 || id>=_initMaxID)
                return -1;
            return _dafidx[daf->getID()];
        }

        /**
         * @brief gets the nr of valid DAFs in the state setup
         * @return nr of valid DAFs
         */
        int getMaxDAFIdx() const {
            return _nrOfDAF;
        }

        /**
         * @brief gets the state structure that the state setup is part
         * of.
         * @return state structure
         */
        const StateStructure* getTree() const {
            return _tree;
        }

        /**
         * @brief gets the state structure that the state setup is part
         * of.
         * @return state structure
         */
        StateStructure* getTree() {
            return _tree;
        }

        /**
         * @brief gets all valid state data of the state setup.
         * @return list of valid state datas
         * @note elements in the list is invalid if they are NULL
         */
        const std::vector<boost::shared_ptr<StateData> >& getStateData() const{
            return _datas;
        }

        inline int getCacheIdx(int id) const { return _sdataTCacheIdx[id]; }

        inline int getMaxCacheIdx() const { return  _nrCaches; }

    private:
        friend class StateData;

        // the version of the State Setup
        int _version;

        // pointer to the complete set of frames
        StateStructure* _tree;

        std::vector<boost::shared_ptr<StateData> > _datas;

        std::vector<Frame*> _dafs;

        ////////////////////////////////// QState stuff
        // Offsets into the QState array.
        // size == <nr of state datas>
        std::vector<int> _offsets;

        // The total sum of the dofs of the frames.
        int _dof;
        // The total nr of dafs
        int _nrOfDAF;
        // The nr of valid frames
        int _nrOfValidFrames;
        // the initial number of max id
        const int _initMaxID;
        // the number of caches
        int _nrCaches;
        ////////////////////////////////// TreeState stuff
        // indexes into the DAF parents, if -1 then no DAF parent exist
        // size == <nr of statedata>
        std::vector<int> _dafidx;

        // indexes into the DAF children array,
        // size==<nr of statedata>
        std::vector<int> _dafChildidx;

        // indexes into the StateCache array,
        // size==<nr of statedata>
        std::vector<int> _sdataTCacheIdx;
    private:
        // You _can_ go around copying StateSetup without memory leaks or other
        // infelicities, but we don't expect to do that so we disallow it.
        StateSetup(const StateSetup&);
        StateSetup& operator=(const StateSetup&);
    };
    /*@}*/
}} // end namespaces


#endif /*RW_KINEMATICS_STATESETUP_HPP*/
