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

#ifndef rwlibs_devices_pa10_PA10_HPP
#define rwlibs_devices_pa10_PA10_HPP

/**
 * @file PA10.hpp
 */

#include <rw/math/Q.hpp>

namespace rwlibs { namespace devices {

    /** @addtogroup devices */
    /*@{*/

    /**
     * @brief Interface for Mitsubishi PA10 driver and virtual device
     */
    class PA10 {
    public:
        /**
         * @brief destructor
         */
        virtual ~PA10() {};
        
        /**
         * @brief Connect to a releases brakes of PA10
         * 
         * Connects to the PA10 and releases the brakes.
         * 
         * After start is successfully called, the user will
         * have to call update no less than every 400ms. 
         * otherwise the robot stops.
         *
         * @param success [out] whether the devices was successfully started
         * @return the current configuration
         */
        virtual rw::math::Q start(bool& success) = 0;

        /**
         * @brief Sets the current threads I/O privileges
         *
         * If the thread calling update is different from the one calling
         * start it is necessary to call initializeThread to set up the
         * permissions.
         */
        virtual void initializeThread() = 0;

        /**
         * @brief Updates the velocity of the robot and returns 
         * the current configuration.
         * @param dq [in] desired joint velocity
         * @return the current joint configuration
         */
        virtual rw::math::Q update(const rw::math::Q& dq) = 0;

        /**
         * @brief Stops the PA10 and lock brakes        
         */
        virtual void stop() = 0;

    protected:
        /**
         * @brief Creates object
         */
        PA10() {}

    private:
        PA10(const PA10&);
        PA10& operator=(const PA10&);
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
