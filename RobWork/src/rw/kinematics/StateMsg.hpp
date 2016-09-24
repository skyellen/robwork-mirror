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


#ifndef RW_KINEMATICS_STATE_HPP
#define RW_KINEMATICS_STATE_HPP

/**
 * @file State.hpp
 */

#include <vector>

#include "QState.hpp"
#include "TreeState.hpp"
#include <rw/common/Ptr.hpp>



namespace rw { namespace kinematics {

    class Frame;
	class StateStructure;
	class StateCache;
	

//    typedef rw::common::Ptr<StateStructure> StateStructurePtr;
//#endif

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief The state of a work cell (or kinematics tree).
     *
     * You need a work cell state in order to calculate forward kinematics for
     * trees of frames.
     *
     * Work cell states can be copied and assigned freely.
     *
     * The work cell state consists of a part for the tree structure and a part
     * for the configuration values. You are encouraged to use the getParent(),
     * getChildren(), getQ() and setQ() utility functions rather than explicitly
     * type, say, state.getQState().getQ(). That way you will have a much easier
     * time of updating the code if we decide to change the way the kinematics
     * data structures are stored (!). Also getQ(state, frame) is shorter to
     * type also.
     *
     * The operation of a work cell state is undefined valid if the tree used
     * for its initialization is modified. (The implementation takes some care
     * to check for this and crashes the program in a controlled way if it takes
     * place.)
     */
    class StateMsg: public State
    {
    public:

        /**
         * @brief Default constructor giving an empty state.
         * Beware that the state is not initialized and that passing this state
         * to a procedure will typically cause a program crash.
         */
        StateMsg();

        /**
         * @brief copies data from a state into this state. The version
         * of the state is allowed to be different from this state. Only
         * state data that is valid for both states will be copied.
         * @param src [in] the state that is to be copied
         */
        void copy(const State &src);

        /**
         * @brief performs a deep copy of \b src into this state.
         * @param src [in] the state that is to be cloned
         */
        void clone( const State& src );


        /**
         * @brief this function upgrades the current version of this
         * State to the newest state. It will not override data values that
         * is set in the current state.
         */
        void upgrade();

        /**
         * @brief this function upgrades the current version of this
         * State with the given state. It will not override data values that
         * is set in the current state.
         */
        void upgradeTo(const State &state)
        {
            State newState = state;
            newState.copy(*this);
            *this = newState;
        }

        /**
         * @brief The dimension of the state vector.
         *
         * Knowing the size of the state is useful for example in error
         * messages, so that you can report if two states seem to belong to
         * different workcells.
         */
        size_t size() const { return getQState().size(); }

        /**
         * @brief Provides direct access to the configurations stored in the state
         *
         * Notice that modifying a state directly may result in the state being inconsistent
         *
         * @param index [in] Index of element to access
         */
        double& operator()(size_t index) { return getQState()(index); }

        /**
         * @brief Provides direct read access to the configurations stored in the state
         *
         * @param index [in] Index of element to access
         */
        const double& operator()(size_t index) const { return getQState()(index); }

        /**
           @brief Same as operator().
         */
        double& operator[](size_t index) { return operator()(index); }

        /**
           @brief Same as operator().
         */
        const double& operator[](size_t index) const { return operator()(index); }

        /**
         * @brief gets the frame with id \b id. If a frame with id \b id does not exist
         * NULL is returned
         */
        Frame* getFrame(int id);

        /**
         * @brief get the state id. Represents the static structure of the StateStructure that
         * this state relates to.
         */
        int getUniqueId() const { return _stateUniqueId; }

        /**
         * @brief Returns pointer to the state structure (the  structure of Frame's and StateData)
         * @return Pointer to the StateStructure matching the frame
         */
		rw::common::Ptr<StateStructure> getStateStructure() const;



    private:
        friend class StateData;
        friend class Frame;
        friend class StateStructure;
        /**
         * @brief The configuration values part of the state.
         */
        const QState& getQState() const { return _q_state; }

        /**
         * @brief The configuration values part of the state.
         */
        QState& getQState() { return _q_state; }

        /**
         * @brief The tree structure part of the state.
         */
        const TreeState& getTreeState() const { return _tree_state; }

        /**
         * @brief The tree structure part of the state.
         */
        TreeState& getTreeState() { return _tree_state; }

        /**
         * @brief the cache part of the state
         * @return vector of caches, the mapping from StateData to cache is located in StateSetup
         */
        //std::vector<rw::common::Ptr<StateCache> >& getCacheState(){return _cache_state;}

        rw::common::Ptr<StateCache> getCache(int id);

        rw::common::Ptr<StateCache> getCache(int id) const;

        void setCache(int id, rw::common::Ptr<StateCache> cache);


        /**
         * @brief Constructs a state
         */
        State(const QState& q_state,
              const TreeState& tree_state,
              int stateUniqueId);

    private:
        TreeState _tree_state;
        QState _q_state;
        std::vector<rw::common::Ptr<StateCache> > _cache_state;
        int _stateUniqueId;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
