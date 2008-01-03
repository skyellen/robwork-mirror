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

#ifndef rw_models_DeviceJacobian_HPP
#define rw_models_DeviceJacobian_HPP

/**
 * @file DeviceJacobian.hpp
 */

namespace rw { namespace math { class Jacobian; }}
namespace rw { namespace kinematics { class State; class FKTable; }}

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
       @brief The Jacobian for the end-effectors of some device.

       The Jacobians are computed relative to the world.

       The Jacobian belongs to a sequence of end-effectors. The dimension of the
       Jacobian returned is (N * 6) x DOF where N is the number of end-effectors
       and DOF is Device::getDOF() for the device.

       The end-effectors and the device for which the Jacobian is computed in
       known from the context from which the DeviceJacobian object was
       constructed.

       An implementation of DeviceJacobian may be valid only for certain changes
       relative to the initial state from which it was constructed. In general
       you may assume only that the computation of the Jacobian is correct only
       for states that differ only in their configuration for the device.
    */
    class DeviceJacobian
    {
    public:
        /**
           @brief The Jacobian for the end-effector frames for a configuration state of
           \b state.
         */
        math::Jacobian get(const kinematics::State& state) const;

        /**
           @brief The Jacobian for the end-effector for a forward kinematics of
           \b fk.
        */
        math::Jacobian get(const kinematics::FKTable& fk) const;

        /**
           @brief Destructor.
         */
        virtual ~DeviceJacobian() {}

    private:
        // These are the methods you should override to implement the get() methods.
        virtual math::Jacobian doGet(const kinematics::FKTable& fk) const = 0;

    protected:
        /**
           @brief Constructor
         */
        DeviceJacobian() {}

    private:
        DeviceJacobian(const DeviceJacobian&);
        DeviceJacobian& operator=(const DeviceJacobian&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
