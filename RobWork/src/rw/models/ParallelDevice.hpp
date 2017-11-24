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

		//! @brief type for a set of legs.
		typedef std::vector<ParallelLeg*> Legs;

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
            const Legs& legs,
            const std::string name,
            const kinematics::State& state);

        /**
         * @brief Constructor for parallel device with multiple junctions.
         * @param name [in] name of the device.
         * @param base [in] the base frame.
         * @param end [in] the end frame.
         * @param joints [in] a list of joints. Each joint can be included in multiple legs.
         * @param state [in] the state used to construct a JointDevice.
         * @param junctions [in] a list of junctions.
         * Each junction is given by a list of legs that must begin and end in the same frame.
         */
        ParallelDevice(
            const std::string name,
			rw::kinematics::Frame* base,
			rw::kinematics::Frame* end,
			const std::vector<Joint*>& joints,
			const rw::kinematics::State& state,
            const std::vector<Legs>& junctions);

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

        /**
         * @brief Set only some of the actuated joints.
         *
         * This version of setQ will only set a subset of the actuated joints.
         * Based on the value of \n q, the function will compute the values for the
         * unactuated (passive) joints, and the remaining actuated joints.
         *
         * This is mainly useful for parallel devices that have more controlled joints
         * than strictly required to make the kinematics determined.
         *
         * @param q [in] the configuration of the actuated joints
         * (the only considered elements are the ones where the corresponding elements of \b enabled is true).
         * @param enabled [in] vector of same size as \b q, specifying which values to solve for.
         * @param state [in/out] the state with all active and passive joint values.
         * The input state is expected to contain a valid and consistent configuration of the device.
         */
        virtual void setQ(const rw::math::Q& q, const std::vector<bool>& enabled, rw::kinematics::State& state) const;

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
         * @brief Get the junctions of the device.
         * @return a vector of junctions. Each junction is given by a two or more legs.
         */
        virtual std::vector<Legs> getJunctions() const { return _junctions; }

        /**
         * @brief The active joints of the parallel device.
         */
        virtual std::vector<models::Joint*> getActiveJoints() const { return _actuatedJoints; }

        /**
         * @brief Get all joints (both active and passive).
         * @return a vector of all the joints.
         */
        virtual std::vector<rw::models::Joint*> getAllJoints() const;

        /**
         * @brief Get the total degrees of freedom for all (active and passive) joints in the device.
         * @return the total degrees of freedom.
         */
        std::size_t getFullDOF() const;

        /**
         * @brief Get bounds for all joints (includes both active and passive joints).
         * @return a pair with the lower and upper limits.
         */
        std::pair<rw::math::Q, rw::math::Q> getAllBounds() const;

        /**
         * @brief Get the full configuration vector of the device. This gives the complete state of the parallel device.
         * @param state [in] the state that contains the full configuration.
         * @return the configuration vector with the joint values for both active and passive joints.
         */
        rw::math::Q getFullQ(const rw::kinematics::State& state) const;

        /**
         * @brief Set the full configuration of the device.
         * This sets the joint values directly, and there is no checks or guarantees that the device
         * will be in a valid connected configuration afterwards.
         * @param q [in] the configuration vector to set.
         * @param state [in/out] the state to update with a new configuration.
         */
        void setFullQ(const rw::math::Q& q, rw::kinematics::State& state) const;

    private:
        static rw::math::Jacobian baseJend(const std::vector<ParallelLeg*>& legs, const kinematics::State& state);

        void normalizeJoints(rw::kinematics::State& state) const;

        std::vector<ParallelLeg*> _legs;
        std::vector<Legs> _junctions;
        //std::vector<kinematics::Frame*> _secondaryRef;

        std::vector<models::Joint*> _actuatedJoints; // list of actuated joints
        std::vector<models::Joint*> _unActuatedJoints; // list of unactuated joints
        std::vector<models::Joint*> _joints; // list of all joints
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
