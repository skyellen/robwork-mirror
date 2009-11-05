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


#ifndef RW_MODELS_JOINTDEVICE_HPP
#define RW_MODELS_JOINTDEVICE_HPP

/**
 * @file JointDevice.hpp
 */

#include "Device.hpp"
#include "JacobianCalculator.hpp"

#include <vector>

namespace rw { namespace models {

    class Joint;


    /** @addtogroup models */
    /*@{*/

    /**
     @brief A device for a sequence of joints.

     Contrary to for example SerialDevice and TreeDevice, the joints need not
     have any particular ordering within the kinematic tree.

     A JointDevice is a joint for which the values of the configuration Q each
     correspond to a frame of type Joint.

     To implement a Device it is common to derive from JointDevice and just
     add implement methods where your device differs from the standard
     behaviour. Subclasses typically differ in their implementation of setQ()
     and the Jacobian computation.
     */
    class JointDevice: public Device
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
        JointDevice(const std::string& name, kinematics::Frame* base,
                    kinematics::Frame* end,
                    const std::vector<Joint*>& joints,
                    const kinematics::State& state);

        // The following are methods specific to JointDevice. The methods are
        // kind of dirty, and should be used with restraint.

        /**
         @brief The active joint at index \b index.

         This method is provided for backward compatibility with SerialDevice
         and TreeDevice.
         */
        //Joint* getActiveJoint(size_t index) const;

        const std::vector<Joint*>& getJoints() const {
            return _joints;
        }
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


        /** @copydoc Device::baseJframes */
       /* math::Jacobian baseJframes(const std::vector<kinematics::Frame*>& frames,
                                   const kinematics::State& state) const;
*/
        /** @copydoc Device::baseJCframes */
        JacobianCalculatorPtr baseJCframes(const std::vector<kinematics::Frame*>& frames,
                                           const kinematics::State& state) const;

        /** @copydoc Device::getBase */
        kinematics::Frame* getBase()
        {
            return _base;
        }

        /** @copydoc Device::getBase */
        const kinematics::Frame* getBase() const
        {
            return _base;
        }

        /** @copydoc Device::getEnd() */
        virtual kinematics::Frame* getEnd()
        {
            return _end;
        }

        /** @copydoc Device::getEnd */
        virtual const kinematics::Frame* getEnd() const
        {
            return _end;
        }

    private:
      /*  const BasicDevice& getBasicDevice() const
        {
            return _bd;
        }
        BasicDevice& getBasicDevice()
        {
            return _bd;
        }
*/
        kinematics::Frame* _base;
        kinematics::Frame* _end;

        std::vector<Joint*> _joints;
        size_t _dof;

        JacobianCalculatorPtr _baseJCend;
        //BasicDevice _bd;
        //BasicDeviceJacobian _dj;
    };


    typedef rw::common::Ptr<JointDevice> JointDevicePtr;

    /*@}*/
}} // end namespaces

#endif // end include guard
