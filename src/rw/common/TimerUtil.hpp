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

#ifndef RW_COMMON_TIMERUTIL_HPP
#define RW_COMMON_TIMERUTIL_HPP

/**
 * @file TimerUtil.hpp
 */

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    /**
     * @brief Access of the system clock
     */
    class TimerUtil
    {
    public:
        /**
         * @brief Returns system clock in milli-seconds
         */
        static long currentTimeMs();


        /**
         * @brief Returns system clock is micro-seconds
         */
        static long currentTimeUs();

        /**
         * @brief Returns system clock in seconds
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
