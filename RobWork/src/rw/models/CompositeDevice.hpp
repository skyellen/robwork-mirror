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


#ifndef RW_MODELS_COMPOSITEDEVICE_HPP
#define RW_MODELS_COMPOSITEDEVICE_HPP

/**
 * @file CompositeDevice.hpp
 */

#include "JointDevice.hpp"
#include <vector>

namespace rw { namespace models {

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
			const std::vector<Device::Ptr>& devices,
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
			const std::vector<Device::Ptr>& devices,
            const std::vector<rw::kinematics::Frame*>& ends,
            const std::string& name,
            const kinematics::State& state);

        //! @brief destructor
        virtual ~CompositeDevice(){}

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
		std::vector<Device::Ptr> _devices;
        std::vector<kinematics::Frame*> _ends;
        rw::common::Ptr<class JacobianCalculator> _djmulti;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
