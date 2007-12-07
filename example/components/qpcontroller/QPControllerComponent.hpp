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

#ifndef rwlibs_components_qpcontroller_QPControllerComponent_HPP
#define rwlibs_components_qpcontroller_QPControllerComponent_HPP

/**
 * @file QPControllerComponent.hpp
 */

#include <cmath>
#include <boost/numeric/ublas/vector.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Event.hpp>
#include <rtt/Command.hpp>
#include <rtt/Properties.hpp>
#include <rtt/TimeService.hpp>

#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>

#include <rwlibs/algorithms/qpcontroller/QPController.hpp>

namespace rwlibs { namespace components {

    /** @addtogroup components */
    /** @{*/

    /**
     * @brief Provides an Orocos TaskContext component wrapping the QPController
     */
    class QPControllerComponent: public RTT::TaskContext
    {
    public:
        /**
         * @brief constructs the component
         * @param name [in] name of component
         * @param h [in] time-step to use in the QPController
         * @param state [in] the state of the workcell
         * @param device [in] the device to control
         */
        QPControllerComponent(
            const std::string& name,
            double h,
            const rw::kinematics::State& state,
            rw::models::DeviceModel* device);

        /**
         * @brief destructor
         */
        virtual ~QPControllerComponent();

    protected:
        /**
         * @brief In port for VelocityScrew6D
         */
        RTT::ReadDataPort<rw::math::VelocityScrew6D<> > _vsIn;

        /**
         * @brief In port for joint configuration \f$ q_{current} \f$
         */
        RTT::ReadDataPort<rw::math::Q > _qIn;

        /**
         * @brief In port for robot joint velocity \f$ \dot{q}_{current} \f$
         */
        RTT::ReadDataPort<rw::math::Q > _qdotIn;

        /**
         * @brief Out port for robot joint velocity \f$ \dot{q}_{new} \f$
         */
        RTT::WriteDataPort<rw::math::Q > _qdotOut;


    private:
        bool startup();
        void update();
        void shutdown();

        rwlibs::algorithms::QPController _qpcontroller;

        rw::models::DeviceModel* _device;
        rw::kinematics::State _state;
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
