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


#ifndef RW_KINEMATICS_STATECACHE_HPP
#define RW_KINEMATICS_STATECACHE_HPP

/**
   @file StateCache.hpp
*/

#include <rw/common/Ptr.hpp>


namespace rw { namespace kinematics {

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief the basic building block for the stateless desing using
     * the StateStructure class. A StateCache represents a size,
     * a unique id, and a unique name, when inserted into the StateStructure.
     * The size will allocate "size"-doubles in State objects originating from the
     * StateStructure.
     */
    class StateCache {
    public:

        //! Smart pointer type
        typedef rw::common::Ptr<StateCache> Ptr;

        /**
         * @brief destructor
         */
        virtual ~StateCache(){ };

        /**
         * @brief An integer ID for the StateCache.
         *
         * IDs are assigned to the state data upon insertion State.
         * StateCache that are not in a State have an ID of -1.
         *
         * StateCache present in different trees may have identical IDs.
         *
         * IDs are used for the efficient implementation of State. Normally,
         * you should not make use of frame IDs yourself.
         *
         * @return An integer ID for the frame.
         */
        //inline int getID() const { return _id; }

        /**
         * @brief The number of doubles allocated by this StateCache in
         * each State object.
         *
         * @return The number of doubles allocated by the StateCache
         */
        virtual size_t size() const = 0;

        /**
         * @brief this creates a deep copy of this cache
         */
        virtual rw::common::Ptr<StateCache> clone() const = 0;

    protected:
        StateCache(){};
        // StateCache should not be copied by other than its inherited class.
        //StateCache(const StateCache&);
        //StateCache& operator=(const StateCache&);
    private:


    };
    /*@}*/
}}

#endif /*RW_KINEMATICS_STATEDATA_HPP*/
