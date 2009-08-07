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


#ifndef RW_ROBP_STATETRAJECTORYFACTORY_HPP
#define RW_ROBP_STATETRAJECTORYFACTORY_HPP

#include "Timed.hpp"

#include <vector>
#include <memory>

namespace rw { namespace kinematics {
    class State;
}}

namespace rw { namespace models {
    class WorkCell;
}}

namespace rw { namespace interpolator {

    /** @addtogroup interpolator */
    /*@{*/

    class StateTrajectory;

    /**
     * @brief StateTrajectory constructors
     */
    class StateTrajectoryFactory
    {
    public:
        /**
         * @brief A trajectory for the straight line path \b path that is linearly
         * traversed with maximum speeds of the devices of \b workcell.
         * The path must be of length at least two.
         */
        static std::auto_ptr<StateTrajectory> makeLinear(
            const models::WorkCell& workcell,
            const std::vector<kinematics::State>& path);

        //! A tuple (time, state).
        typedef Timed<kinematics::State> TimedState;

        //! A sequence of tuples of (time, state).
        typedef std::vector<TimedState> TimedStatePath;

        /**
         * @brief A trajectory for the straight line path \b path that is
         * linearly traversed to match the provided time values.
         *
         *  The path must be of length at least two.
         */
        static std::auto_ptr<StateTrajectory> makeLinear(const TimedStatePath& path);

        /**
         * @brief A trajectory containing no states.
         *
         *  The end time of the trajectory is negative. Calling the get() method will
         *  throw an exception always, because the trajectory range is empty.
         */
        static std::auto_ptr<StateTrajectory> makeEmpty();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
