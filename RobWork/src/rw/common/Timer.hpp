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


#ifndef RW_COMMON_TIMER_HPP
#define RW_COMMON_TIMER_HPP

#include <string>
#include "TimerUtil.hpp"

/**
 * @file Timer.hpp
 */

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    /**
     * @brief The timer class provides an easy to use platform independent timer
     *
     * In Windows the expected resolution is approx. 16ms.
     * In Linux the expected resolution is approx. 1ms
     */
    class Timer
    {
    public:
        /**
         * @brief Constructor
         *
         * This implicitly starts the timer.
         */
        Timer();

        /**
         * @brief constructor - initialize the timer to a specified value. This does not start the timer.
         * @param timems [in] time in ms
         */
        Timer(long timems);

        /**
         * @brief constructor - initialize the timer to a specified value. This does not start the timer.
         * @param hh [in] hours
         * @param mm [in] minutes
         * @param ss [in] seconds
         * @param ms [in] milli seconds
         */
        Timer(int hh, int mm, int ss = 0, int ms = 0);

        /**
         * @brief Destructor
         */
        virtual ~Timer();

		/** 
		 * @brief Returns true if the timer is paused
		 * @return True is paused
		 */
		bool isPaused();
 
        /**
         * @brief Reset the timer
         *
         * The timer is set back to zero and starts counting.
         *
         * It is OK to call reset() on a timer that has already been started:
         * The time will just be set back to zero again.
         */
        void reset();

    
        /**
         * @brief Resets and pauses the timer
         *
         * The timer is set to zero and paused
         *
         * It is OK to call reset() on a timer that has already been started:
         * The time will just be set back to zero again.
         */
        void resetAndPause();

        /**
         * @brief Resets and stats the timer
         *
         * Same as reset()
         *
         * The timer is set to zero and starts counting
         *
         * It is OK to call reset() on a timer that has already been started:
         * The time will just be set back to zero again.
         */
        void resetAndResume();

        /**
         * @brief Pause the timer
         *
         * The timer stops counting. As long as the timer has not been resumed
         * (see resume()) or restarted (see reset()) the timer value (see
         * getTime()) will stay the same.
         *
         * Is is OK to call pause() on a timer that has already been paused: The
         * timer will just stay paused and nothing is changed.
         */
        void pause();

        /**
         * @brief Resume the timer after a pause.
         *
         * The timer starts counting again.
         *
         * It is OK to call resume() on a timer that is already counting: The
         * timer keeps counting and nothing is done.
         */
        void resume();

        /**
         * @brief The time the timer has been running.
         *
         * The time passed while the timer has been paused are not included in
         * the running time. The timer is paused when pause() has been
         * called and counting is resumed when resume() has been called.
         *
         * It is perfectly OK and natural to call getTime() on a running timer,
         * i.e. a timer that has not been paused.
         *
         * A call of reset() resets the running time to zero.
         *
         * \return Time in seconds
         */
        double getTime() const;

        /**
         * @brief The time the timer has been running in hole seconds.
         *
         * see getTime
         *
         * \return Time in hole seconds
         */
        long getTimeSec() const;

        /**
         * @brief The time the timer has been running in mili seconds.
         *
         * see getTime
         *
         * \return Time in mili seconds
         */
        long getTimeMs() const;


        /**
         * @brief returns a string describing the time. The format of the time is described using \b format
         * @param format [in] the format is on the form:
         *  hh:mm:ss --> 05:06:08
         *  h:m:s --> 5:6:8
         * @return a formated time string
         */
        std::string toString(const std::string& format="hh:mm:ss");


        /**
         * @brief Returns system clock in hole seconds
		 *
		 * \warning The date/time at which this timer counts from is platform-specific, so
		 * you should \b not use it for getting the calendar time. It's really only meant for
		 * calculating wall time differences.
		 */
        static long currentTimeSec(){ return (long)(TimerUtil::currentTimeMs()/1000); }


        /**
         * @brief Returns system clock in milli-seconds
		 *
		 * \warning The date/time at which this timer counts from is platform-specific, so
		 * you should \b not use it for getting the calendar time. It's really only meant for
		 * calculating wall time differences.
		 */
        static long currentTimeMs(){ return (long)TimerUtil::currentTimeMs(); }


		/**
		 * @brief Returns system clock in micro-seconds.
         *
		 * \warning The date/time at which this timer counts from is platform-specific, so
		 * you should \b not use it for getting the calendar time. It's really only meant for
		 * calculating wall time differences.
         *
         * Notice: The timer cannot hold times longer than approx. 2100second.
		 */
        static long currentTimeUs(){ return (long)TimerUtil::currentTimeUs(); }

        /**
         * @brief Returns system clock in seconds
		 *
		 * \warning The date/time at which this timer counts from is platform-specific, so
		 * you should \b not use it for getting the calendar time. It's really only meant for
		 * calculating wall time differences.
		 */
        static double currentTime(){ return TimerUtil::currentTime(); }


        /**
         * @brief Sleeps for a period of time
         *
         * @param period [in] the time in miliseconds to sleep
         */
        static void sleepMs(int period){ TimerUtil::sleepMs(period); }

        /**
         * @brief Sleeps for a period of time
         *
         * @param period [in] the time in microseconds to sleep
         */
        static void sleepUs(int period){ TimerUtil::sleepUs(period); }


    private:
        // Total time (in mili seconds).
        long _totalTime;

        // Time of the last resume (or reset).
        long _relativeTime;

        // True iff the timer is paused.
        bool _isPaused;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
