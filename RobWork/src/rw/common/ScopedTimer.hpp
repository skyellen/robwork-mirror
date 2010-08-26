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

#ifndef RW_COMMON_SCOPEDTIMER_HPP
#define RW_COMMON_SCOPEDTIMER_HPP

#include <rw/common/Timer.hpp>

namespace rw {
namespace common {

    /**
     * @brief Times what is executed in a scope.
     *
	 * Automatically calls resume on the timer given in construction and pause when destroyed.
	 *
     * @note usage
     * \code
     * ...
     * long time;
     * {
     *   // put code here that is not to be timed
     *   ScopedTimer timer(time);
     *   // put code here that is to be timed
     * }
     * std::cout << "Time: " << time << std::endl;
     * \endcode
     */
    class ScopedTimer {
    public:
        /**
         * @brief constructor. Starts the timer
         * @param timer         
         */
        ScopedTimer(Timer& timer): _timer(timer) {
        	_timer.resume();
        }

        /**
         * @brief destructor, stops the timer         
         */
        virtual ~ScopedTimer(){
            _timer.pause();
        }

        /**
         * @brief Returns the time wrapped in the ScopedTimer
         * @return Timer wrapped
         */
        Timer& getTimer() {
            return _timer;
        }

    private:
        Timer& _timer;
    };

}
}


#endif //End include guard
