/*********************************************************************
 * RobWork Version 0.2
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

#ifndef rwlibs_devices_pa10_PA10Virtual_HPP
#define rwlibs_devices_pa10_PA10Virtual_HPP

/**
 * @file PA10Virtual.hpp
 */

#include "PA10.hpp"

#include <rw/math/Q.hpp>
#include <rw/common/Timer.hpp>

#include <fstream>

namespace rwlibs { namespace devices {

    /** @addtogroup devices */
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
}} // end namespaces

#endif // end include guard
