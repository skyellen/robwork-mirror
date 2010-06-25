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

#ifndef RWHW_PA10VIRTUAL_HPP
#define RWHW_PA10VIRTUAL_HPP

/**
 * @file PA10Virtual.hpp
 */

#include "PA10.hpp"

#include <rw/math/Q.hpp>
#include <rw/common/Timer.hpp>

#include <fstream>

namespace rwhw {

    /** @addtogroup pa10 */
    /*@{*/

    /**
     * @brief Implements virtual Mitsubishi PA10
     */
    class PA10Virtual: public PA10
    {
    public:
        /**
         * @brief Creates object
         * @param q [in] Initial configuration of device. Default value is 0
         */
        PA10Virtual(const rw::math::Q& q = rw::math::Q(rw::math::Q::ZeroBase(7)));

        /**
         * @copydoc PA10::start
         */
        rw::math::Q start(bool& success);

        /**
         * @copydoc PA10::initializeThread
         *
         * @note Does nothing for PA10Virtual
         */
        void initializeThread();

        /**
         * @copydoc PA10::update
         */
        rw::math::Q update(const rw::math::Q& dq);

        /**
         * @copydoc PA10::stop
         */
        void stop();

    private:
        rw::math::Q _q;
        rw::math::Q _dqlast;
        rw::common::Timer _timer;
    };

    /**@}*/
} // end namespaces

#endif // end include guard
