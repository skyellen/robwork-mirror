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

#ifndef rwlibs_components_pa10_PA10Component_HPP
#define rwlibs_components_pa10_PA10Component_HPP

/**
 * @file PA10Component.hpp
 */

#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>

#include <rwlibs/devices/pa10/PA10.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Event.hpp>
#include <rtt/Command.hpp>
#include <rtt/Properties.hpp>
#include <rtt/TimeService.hpp>

#include <fstream>

namespace rwlibs { namespace components {

    /** @addtogroup components */
    /*@{*/

    /**
     * @brief Implements a TaskContext communucation with a Mitsubishi PA10
     */
    class PA10Component : public RTT::TaskContext {

    public:
        /**
         * @brief Creates object
         */
        PA10Component(rwlibs::devices::PA10* pa10, double dt);

    protected:
        /**
         * @brief A vector containing the input values of the position sensors
         *
         * Other components can use this port for robot control
         */
        RTT::DataPort<rw::math::Q> _qCurrent;

        /**
         * @brief DataPort containing the lastest velocity
         */
        RTT::DataPort<rw::math::Q> _qdotCurrent;

        /**
         * @brief a vector containing the output velocities to the joint controllers
         *
         * Other components can use this port for robot control
         */
        RTT::DataPort<rw::math::Q> _qdotTarget;

        /**
         * @brief Port required to be false for the robot to run
         */
        RTT::DataPort<bool> _errorPort;

    private:
        bool startup();
        void update();
        void shutdown();

        rwlibs::devices::PA10* _pa10;
        bool _initialized;
        double _dt;
    };

    /* @} */

}} // end namespaces

#endif // end include guard
