/*
 * ScopedTimer.hpp
 *
 *  Created on: 01-12-2008
 *      Author: jimali
 */

#ifndef RW_COMMON_SCOPEDTIMER_HPP
#define RW_COMMON_SCOPEDTIMER_HPP

#include <rw/common/Timer.hpp>

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
         * @param timer
         * @return
         */
        ScopedTimer(Timer& timer): _timer(timer) {
        	_timer.resume();
        }

        /**
         * @brief destructor, stops the timer
         * @return
         */
        virtual ~ScopedTimer(){
            _timer.pause();
        }

    private:
        Timer& _timer;
    };

}
}


#endif //End include guard
