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


#ifndef RW_ROBP_TIMED_HPP
#define RW_ROBP_TIMED_HPP

/**
   @file Timed.hpp
   @brief Class rw::interpolator::Timed
*/

#include <vector>

namespace rw { namespace interpolator {

    /** @addtogroup interpolator */
    /*@{*/

    /**
       @brief A tuple of (time, value).
     */
    template <class T>
    class Timed
    {
    public:
        /**
           Constructor
         */
        Timed(double time, const T& value) :
            _time(time),
            _value(value)
        {}

        /**
           @brief The time
        */
        double getTime() const { return _time; }

        /**
           @brief The value
        */
        const T& getValue() const { return _value; }

    private:
        double _time;
        T _value;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
