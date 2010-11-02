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


#ifndef RW_MODELS_PARALLELDEVICE_HPP
#define RW_MODELS_PARALLELDEVICE_HPP

/**
 * @file ParallelDevice.hpp
 */

#include "JointDevice.hpp"

#include <vector>
#include <string>

namespace rw { namespace math { class Jacobian; }}
namespace rw { namespace kinematics { class State; class Frame; }}

namespace rw { namespace models {

    class Joint;
    class ParallelLeg;

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief This class defines the interface for Parallel devices.
     */
    class ParallelDevice : public JointDevice
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<ParallelDevice> Ptr;

        /**
         * @brief Constructor
         *
         * @param legs [in] the serial legs connecting the endplate to the base.
         * The base of each serial Leg must be the same frame. Likewise, the endeffector
         * (last frame) of each Leg must transform to the same transform as each of the
         * other legs
         * @param name [in] name of device
         * @param state [in] the state for the assembly mode
         */
        ParallelDevice(
            const std::vector<ParallelLeg*>& legs,
            const std::string name,
            const kinematics::State& state);

        /** @brief Destructor */
        ~ParallelDevice();

        /**
         * @copydoc Device::setQ
         *
         * The configuration \b q is the configuration for the actuated joints
         * of the parallel device. Based on the value of \b q the setQ() method
         * automatically computes the values for the unactuated (passive)
         * joints.
         */
        virtual void setQ(const math::Q& q, kinematics::State& state) const;

        /** @copydoc Device::baseJframe */
        math::Jacobian baseJframe(
            const kinematics::Frame* frame,
            const kinematics::State& state) const;

        /** @copydoc Device::baseJend */
        math::Jacobian baseJend(const kinematics::State& state) const;

        /**
         * @brief The legs of the parallel device.
         */
        virtual std::vector<ParallelLeg*> getLegs() const { return _legs;}

        /**
         * @brief The active joints of the parallel device.
         */
        virtual std::vector<models::Joint*> getActiveJoints()
        { return _actuatedJoints; }

    private:
        std::vector<ParallelLeg*> _legs;
        std::vector<kinematics::Frame*> _secondaryRef;

        std::vector<models::Joint*> _actuatedJoints; // list of actuated joints
        std::vector<models::Joint*> _unActuatedJoints; // list of unactuated joints

        boost::numeric::ublas::matrix<double>* _aJointJ; // the actuated joint jacobian
        boost::numeric::ublas::matrix<double>* _uaJointJ; // the unactuated joint jacobian

        math::Q* _lastAJVal; // current actuated joint values
        math::Q* _lastUAJVal;// current unactuated joint values
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
