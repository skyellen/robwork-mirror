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


#ifndef RW_COMMON_TIMERUTIL_HPP
#define RW_COMMON_TIMERUTIL_HPP

/**
 * @file TimerUtil.hpp
 */

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    /**
     * @brief Access of the system clock so called wall time.
     */
    class TimerUtil
    {
    public:
        /**
         * @brief Returns system clock in milli-seconds
		 *
		 * \warning The date/time at which this timer counts from is platform-specific, so
		 * you should \b not use it for getting the calendar time. It's really only meant for 
		 * calculating wall time differences.
		 */
        static long long currentTimeMs();


		/**
		 * @brief Returns system clock in micro-seconds. 		         
         *
		 * \warning The date/time at which this timer counts from is platform-specific, so
		 * you should \b not use it for getting the calendar time. It's really only meant for 
		 * calculating wall time differences.
         *
         * Notice: The timer cannot hold times longer than approx. 2100second.
		 */
        static long long currentTimeUs();

        /**
         * @brief Returns system clock in seconds
		 *
		 * \warning The date/time at which this timer counts from is platform-specific, so
		 * you should \b not use it for getting the calendar time. It's really only meant for 
		 * calculating wall time differences.
		 */
        static double currentTime();


        /**
         * @brief Sleeps for a period of time
         *
         * @param period [in] the time in miliseconds to sleep
         */
        static void sleepMs(int period);

        /**
         * @brief Sleeps for a period of time
         *
         * @param period [in] the time in microseconds to sleep
         */
        static void sleepUs(int period);

    };

    /* @} */
}} // end namespace

#endif // end include guard
