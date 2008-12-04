/*
 * ScopedTimer.hpp
 *
 *  Created on: 01-12-2008
 *      Author: jimali
 */

#ifndef RW_COMMON_SCOPEDTIMER_HPP_
#define RW_COMMON_SCOPEDTIMER_HPP_

#include <rw/common/TimerUtil.hpp>

namespace rw {
namespace common {

    /**
     * @brief times what is executed in a scope.
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
         * @param time
         * @return
         */
        ScopedTimer(long &time):_totalTime(time){
            _start = TimerUtil::currentTimeMs();
        }

        /**
         * @brief destructor, stops the timer
         * @return
         */
        virtual ~ScopedTimer(){
            _end = TimerUtil::currentTimeMs();
            _totalTime = _end-_start;
        }

        /**
         * @breif gets the current time in ms since the construction of
         * the scoped timer
         */
        long getTimeMs(){
            long end = TimerUtil::currentTimeMs();
            return end-_start;
        }
    private:
        long &_totalTime;
        long _start, _end;
    };

}
}


#endif /* SCOPEDTIMER_HPP_ */
