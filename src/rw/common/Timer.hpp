/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

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
