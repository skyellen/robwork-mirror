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


#ifndef RW_ROBP_STATETRAJECTORY_HPP
#define RW_ROBP_STATETRAJECTORY_HPP

#include <vector>

namespace rw { namespace kinematics { class State; }}

namespace rw { namespace interpolator {

    /** @addtogroup interpolator */
    /*@{*/

    /**
       @brief A trajectory of configurations.

       A trajectory is a mapping from the time interval [0, end] into the
       space of work cell states.

       The mapping won't necessarily be continuous. For example, the state
       trajectory may change the attachment of DAFs.
    */
    class StateTrajectory
    {
    public:
        /**
           @brief The configuration for time \b time.

           An exception is thrown if \b time is out of range.
        */
        virtual kinematics::State get(double time) const = 0;

        /**
           @brief The end time of the mapped time interval.

           The start time is zero always.
        */
        virtual double getEndTime() const = 0;

        /**
           @brief Destructor.
        */
        virtual ~StateTrajectory() {}

    protected:
        /**
           @brief Constructor.
         */
        StateTrajectory() {}

    private:
        StateTrajectory(const StateTrajectory&);
        StateTrajectory& operator=(const StateTrajectory&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
