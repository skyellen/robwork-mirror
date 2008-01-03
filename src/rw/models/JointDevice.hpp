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

#ifndef rw_models_JointDevice_HPP
#define rw_models_JointDevice_HPP

/**
 * @file JointDevice.hpp
 */

#include "Device.hpp"
#include "BasicDevice.hpp"
#include "BasicDeviceJacobian.hpp"

#include <vector>

namespace rw { namespace models {

    class Joint;

    /** @addtogroup models */
    /*@{*/

    /**
       @brief A device for a sequence of joints.

       The JointDevice class provides functionality common to many subclasses of
       Device. To implement a Device it typically suffices to derive from
       JointDevice and just add implement methods where your device differs from
       the standard behaviour. Subclasses typically differ in their
       implementation of setQ() and the Jacobian computation.

       Every standard device and its Jacobian is implemented in terms of a
       BasicDevice. This BasicDevice can be accessed via
       JointDevice::getBasicDevice(). You should use the access to this
       BasicDevice with some constraint as this is an implementation detail
       possibly subject to change.
     */
    class JointDevice : public Device
    {
    public:
        /**
           @brief Construct the device for a sequence of joints.

           @param name [in] name of device

           @param base [in] the base of the device

           @param end [in] the end (or tool) of the device

           @param joints [in] the joints of the device

           @param state [in] the state that shows how frames are connected as
           needed for the computation of Jacobians.
         */
        JointDevice(
            const std::string& name,
            kinematics::Frame* base,
            kinematics::Frame* end,
            const std::vector<Joint*>& joints,
            const kinematics::State& state);

        // The following are methods specific to JointDevice. The methods are
        // more or less dirty, and should be used with restraint.

        /**
           @brief The basic device in terms of which the standard device has
           been implemented.
        */
        const BasicDevice& getBasicDevice() const;

        /**
           @brief The basic device in terms of which the standard device has
           been implemented.
        */
        BasicDevice& getBasicDevice();

        /**
           @brief The active joint at index \b index.

           This method is provided for backward compatibility with SerialDevice
           and TreeDevice. The method is trivially implemented in terms of
           getBasicDevice().
        */
        Joint* getActiveJoint(size_t index) const;

        /**
           @brief A BasicDeviceJacobian for a sequence of frames.

           The Jacobian is computed by calling BasicDeviceJacobian::get().

           The method is provided for backward compatibility with SerialDevice
           and TreeDevice.

           Like the Jacobian methods of Device, this method should probably be
           declared virtual at some point.
         */
        boost::shared_ptr<BasicDeviceJacobian> baseJframes(
            const std::vector<kinematics::Frame*>& frames,
            const kinematics::State& state) const;        

        // Everything below are methods of Device.

        /** @copydoc Device::setQ */
        void setQ(const math::Q& q, kinematics::State& state) const;

        /** @copydoc Device::getQ */
        math::Q getQ(const kinematics::State& state) const;

        /** @copydoc Device::getDOF */
        size_t getDOF() const;

        /** @copydoc Device::getBounds */
        std::pair<math::Q, math::Q> getBounds() const;

        /** @copydoc Device::setBounds */
        void setBounds(const std::pair<math::Q, math::Q>& bounds);

        /** @copydoc Device::getVelocityLimits */
        math::Q getVelocityLimits() const;

        /** @copydoc Device::setVelocityLimits */
        void setVelocityLimits(const math::Q& vellimits);

        /** @copydoc Device::getAccelerationLimits */
        math::Q getAccelerationLimits() const;

        /** @copydoc Device::setAccelerationLimits */
        void setAccelerationLimits(const math::Q& acclimits);

        /** @copydoc Device::baseJend */
        math::Jacobian baseJend(const kinematics::State& state) const;

        /** @copydoc Device::baseJframe */
        math::Jacobian baseJframe(
            const kinematics::Frame *frame,
            const kinematics::State& state) const;

        /** @copydoc Device::getBase */
        kinematics::Frame* getBase() { return _base; }

        /** @copydoc Device::getBase */
        const kinematics::Frame* getBase() const { return _base; }

        /** @copydoc Device::getEnd() */
        virtual kinematics::Frame* getEnd() { return _end; }

        /** @copydoc Device::getEnd */
        virtual const kinematics::Frame* getEnd() const { return _end; }

    private:
        kinematics::Frame* _base;
        kinematics::Frame* _end;
        BasicDevice _bd;
        BasicDeviceJacobian _dj;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
