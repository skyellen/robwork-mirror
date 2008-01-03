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

#ifndef rw_models_BasicDeviceJacobian_HPP
#define rw_models_BasicDeviceJacobian_HPP

/**
 * @file BasicDeviceJacobian.hpp
 */

#include "DeviceJacobian.hpp"
#include "BasicDevice.hpp"

namespace rw { namespace math { class Jacobian; }}
namespace rw { namespace kinematics {
    class Frame;
    class State;
    class FKTable;
}}

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
       @brief The Jacobian for a basic device (BasicDevice).

       The Jacobians are computed relative to the world.
    */
    class BasicDeviceJacobian : public DeviceJacobian
    {
    public:
        /**
           @brief Jacobian computation for the tool \b tcp connected to the
           device \b device for a tree structure of \b state.

           The assumption for the Jacobian computation is that the structure of
           the tree (i.e. the attachment of DAFs) is the same throughout the use
           of BasicDeviceJacobian. If you want to for example attach \b tcp to a
           new frame, you must construct a new BasicDeviceJacobian for this
           particular tree structure state.
        */
        BasicDeviceJacobian(
            const BasicDevice& device,
            const kinematics::Frame* tcp,
            const kinematics::State& state);

        /**
           @brief Jacobian computation for a number of tools \b tcp's connected to the
           device \b device for a tree structure of \b state.

           The assumption for the Jacobian computation is that the structure of
           the tree (i.e. the attachment of DAFs) is the same throughout the use
           of BasicDeviceJacobian. If you want to for example attach \b tcp to a
           new frame, you must construct a new BasicDeviceJacobian for this
           particular tree structure state.

           The dimension of the jacobian wil be (tcps.size() * 6, device.getDOF()).
        */
        BasicDeviceJacobian(
            const BasicDevice& device,
            const std::vector<kinematics::Frame*>& tcps,
            const kinematics::State& state);

        /**
           @brief Destructor.
         */
        ~BasicDeviceJacobian();

    private:
        math::Jacobian doGet(const kinematics::FKTable& fk) const;

    private:
        class Impl;
        Impl* _impl;

    private:
        BasicDeviceJacobian(const BasicDeviceJacobian&);
        BasicDeviceJacobian& operator=(const BasicDeviceJacobian&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
