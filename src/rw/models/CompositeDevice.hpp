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

#ifndef rw_models_CompositeDevice_HPP
#define rw_models_CompositeDevice_HPP

/**
 * @file CompositeDevice.hpp
 */

#include "JointDevice.hpp"

#include <vector>

#include <boost/shared_ptr.hpp>

namespace rw { namespace models {

    class Joint;

    /** @addtogroup models */
    /*@{*/

    /**
       @brief A device constructed from a sequence of devices.

       The configuration of a composite device is equal to the concatenation of
       the configurations of the sequence of devices.

       The devices that make up the CompositeDevice may not share joints, but
       the implementation does not check if this is actually the case.

       A composite device implements its operations of Device by querying each
       Joint in the straight-forward way of JointDevice. The notable
       exception is Device::setQ() which is implemented by forwarding the
       Device::setQ() calls to the sequence of devices. This means that
       CompositeDevice works also for example for devices of type ParallelDevice
       that have an overwritten implementation of Device::setQ().

       The devices from which the composite device is constructed must all be of
       type JointDevice. An exception is thrown by the constructor if one of
       the devices is not of this subtype.

       The computation of Jacobians of CompositeDevice is not correct in
       general, but is correct only for devices for which the standard technique
       of JointDevice is correct. We cannot in general in RobWork do any
       better currently. The implementation does not check if the requirements
       for the computation of Jacobians are indeed satisfied.

       CompositeDevice is related to TreeDevice in the sense that
       CompositeDevice has also multiple end-effectors (one end-effector for
       each device). CompositeDevice differs from TreeDevice by not requiring
       that the child-to-parent paths of the end-effectors connect to a common
       base.
    */
    class CompositeDevice : public JointDevice
    {
    public:
        /**
           @brief Constructor

           @param base [in] the base of the device
           @param devices [in] the sequence of subdevices
           @param end [in] the end (or tool) of the device
           @param name [in] the name of the device
           @param state [in] the kinematic structure assumed for Jacobian computations
        */
        CompositeDevice(
            rw::kinematics::Frame* base,
            const std::vector<DevicePtr>& devices,
            rw::kinematics::Frame* end,
            const std::string& name,
            const kinematics::State& state);

        /**
           @brief Constructor

           @param base [in] the base of the device
           @param devices [in] the sequence of subdevices
           @param ends [in] the end frames (or tools) of the device
           @param name [in] the name of the device
           @param state [in] the kinematic structure assumed for Jacobian computations
        */
        CompositeDevice(
            rw::kinematics::Frame *base,
            const std::vector<DevicePtr>& devices,
            const std::vector<rw::kinematics::Frame*>& ends,
            const std::string& name,
            const kinematics::State& state);

        /**
           @copydoc Device::setQ

           The method is implemented via forwarding to the Device::setQ()
           methods of the subdevices.
        */
        void setQ(const math::Q& q, kinematics::State& state) const;

        // Methods specific to CompositeDevice follow here.

        /**
           @brief like Device::baseJend() but with a Jacobian calculated for all
           end-effectors (see getEnds()).
        */
        math::Jacobian baseJends(const kinematics::State& state) const;

        /**
           @brief The end-effectors of the composite device.

           The end-effectors of the composite device are the end-effectors of
           the devices from which the composite device was constructed.

           This sequence of end-effectors may or may not include the default
           end-effector returned by getEnd().
        */
        const std::vector<kinematics::Frame*>& getEnds() const { return _ends; }

    private:
        std::vector<DevicePtr> _devices;
        std::vector<kinematics::Frame*> _ends;
        boost::shared_ptr<DeviceJacobian> _djmulti;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
