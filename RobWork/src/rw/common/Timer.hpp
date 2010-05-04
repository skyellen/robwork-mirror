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
         * @brief Destructor
         */
        virtual ~Timer();

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
