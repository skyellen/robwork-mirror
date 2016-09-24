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


#ifndef RW_KINEMATICS_STATEDATA_HPP
#define RW_KINEMATICS_STATEDATA_HPP

/**
   @file StateData.hpp
*/

#include <string>

#include "State.hpp"
#include "StateCache.hpp"

namespace rw { namespace kinematics {

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief the basic building block for the stateless design using
     * the StateStructure class. A StateData represents a size,
     * a unique id, and a unique name, when inserted into the StateStructure.
     * The size will allocate "size"-doubles in State objects originating from the
     * StateStructure.
     */
    class StateData {

    public:
        /**
         * @brief destructor
         */
        virtual ~StateData();

        /**
         * @brief An integer ID for the StateData.
         *
         * IDs are assigned to the state data upon insertion State.
         * StateData that are not in a State have an ID of -1.
         *
         * StateData present in different trees may have identical IDs.
         *
         * IDs are used for the efficient implementation of State. Normally,
         * you should not make use of StateData IDs yourself.
         *
         * @return An integer ID for the frame.
         */
        inline int getID() const { return _id; }

        /**
         * @brief The name of the state data.
         *
         * @return The name of the state data.
         */
        const std::string& getName() const { return _name; }

        /**
         * @brief The number of doubles allocated by this StateData in
         * each State object.
         *
         * @return The number of doubles allocated by the StateData
         */
        inline int size() const { return _size; };

        // The StateData values.
        /**
         * @brief An array of length size() containing the values for
         * the state data.
         *
         * It is OK to call this method also for a StateData with zero size.
         *
         * @param state [in] The state containing the StateData values.
         *
         * @return The values for the frame.
         */
        inline const double* getData(const State& state) const {
            if( _size==0 ) return NULL; // stop early if we know size is 0
            if( _id<0 )
            	RW_THROW("StateData \"" << _name << "\" NOT initialized!");
            return state.getQState().getQ(*this);
        }

        /**
         * @brief An array of length size() containing the values for
         * the state data.
         *
         * It is OK to call this method also for a StateData with zero size.
         *
         * @param state [in] The state containing the StateData values.
         *
         * @return The values for the frame.
         */
        inline double* getData(State& state) {
            if( _size==0 ) return NULL; // stop early if we know size is 0
            if( _id<0 )
            	RW_THROW("StateData \"" << _name << "\" NOT initialized!");
            return state.getQState().getQ(*this);
        }

        /**
         * @brief Assign for \b state data the size() of values of the array \b
         * vals.
         *
         * The array \b vals must be of length at least size().
         *
         * @param state [inout] The state to which \b vals are written.
         *
         * @param vals [in] The joint values to assign.
         *
         * setData() and getData() are related as follows:
         * \code
         * data.setData(state, q_in);
         * const double* q_out = data.getData(state);
         * for (int i = 0; i < data.getDOF(); i++)
         *   q_in[i] == q_out[i];
         * \endcode
         */
        inline void setData(State& state, const double* vals) const{
            if( _size==0 ) return; // stop early if we know size is 0
            if( _id<0 ) RW_THROW("StateData \"" << _name << "\" NOT initialized!");
            state.getQState().setQ(*this, vals);
        }

        /**
         * @brief Check is state data includes a cache.
         * @return true if cache, false otherwise.
         */
        inline bool hasCache() const { return _hasCache; }

        //StateData(int size, StateCache::Ptr defaultCache, const std::string& name);
        /**
         * @brief Get the cache.
         * @param state [in] the state.
         * @return the cache.
         */
        rw::common::Ptr<StateCache> getCache(const State& state) const ;

        //! @copydoc getCache(const State&) const .
        rw::common::Ptr<StateCache> getCache(State& state);

        /**
         * @brief Get default cache.
         * @return the cache.
         */
        rw::common::Ptr<StateCache> getDefaultCache(){ return _cache; }

        /**
         * @brief Set the cache values.
         * @param cache [in] the cache.
         * @param state [in/out] state updated with new cache.
         */
        void setCache(rw::common::Ptr<StateCache> cache, State& state);

        /**
         * @brief Get the state structure.
         * @return the state structure.
         */
        class StateStructure* getStateStructure() { return _sstructure;};

    public:

        /**
         * @brief A state with \b size number of doubles in the State vector.
         *
         * \b size must be non-negative.
         *
         * The newly created state data can be added to a structure with
         * StateStructure::addData().
         *
         * The size of the state data in nr of doubles of the state data
         * is constant throughout
         * the lifetime of the state data.
         *
         * @param size [in] The number of degrees of freedom of the frame.
         *
         * @param name [in] The name of the frame.
         */
        StateData(int size, const std::string& name);

        /**
         * @copydoc StateData(int, const std::string&)
         * @param cache [in] a cache.
         */
        StateData(int size, const std::string& name, rw::common::Ptr<StateCache> cache);


    private:
        // The tree is responsible for the assignment of the IDs that are later
        // used in the State implementation. Tree is a friend so that IDs can
        // be modified only from there. The common advice is that the friend
        // class should be in the same header file as this class, but we break
        // that advice to allow the Tree declaration to be excluded from the
        // Doxygen documentation.
        friend class StateStructure;

        void setID(int id, class StateStructure* sstructure) { _id = id; _sstructure = sstructure;}

    private:

        // An integer ID for the frame. Assignment of ID values is the
        // responsibility of the tree in which the frame is inserted. The ID of
        // a frame may change over time.
        int _id;
        // the statestructure in which this statedata is registered
        StateStructure* _sstructure;

        // The size of the state memmory allocated for this state data.
        // This value remains fixed throughout the life time of the statedata.
        const int _size;

        // The name of the state data.
        std::string _name;

        bool _hasCache;

        rw::common::Ptr<StateCache> _cache;

    private:
        // StateData should not be copied.
        StateData(const StateData&);
        StateData& operator=(const StateData&);

    };
    /*@}*/
}}

#endif /*RW_KINEMATICS_STATEDATA_HPP*/
