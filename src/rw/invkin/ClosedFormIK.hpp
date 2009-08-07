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


#ifndef RW_INVKIN_CLOSEFORMIK_HPP
#define RW_INVKIN_CLOSEFORMIK_HPP

/**
 * @file ClosedFormIK.hpp
 */

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/models/Device.hpp>

#include <vector>

namespace rw { namespace invkin {

    /** \addtogroup invkin */
    /*@{*/

    class ClosedFormIK;

    //! A pointer to a ClosedFormIK solver.
    typedef rw::common::Ptr<ClosedFormIK> ClosedFormIKPtr;

    /**
     * @brief Interface for closed form inverse kinematics algorithms.
     *
     * The ClosedFormIK interface provides an interface for calculating the
     * inverse kinematics of a device. That is to calculate the solutions
     * \f$\mathbf{q}_i, i=0,\ldots,\f$, such that
     * \f$\robabx{base}{end}{\mathbf{T}}(\mathbf{q}_i)=
     * \robabx{}{desired}{\mathbf{T}}\f$.
     *
     * By default it solves the problem beginning at the robot base and
     * ending with the frame defined as the end of the devices, and which is
     * accessible through the Device::getEnd() method.
     */
    class ClosedFormIK
    {
    public:
        /**
         * @brief Calculates the inverse kinematics solutions
         *
         * Given a desired \f$\robabx{}{desired}{\mathbf{T}}{}\f$ the method
         * solves the inverse kinematics problem, and provides a list of valid
         * configuration. That is only configurations which are within the
         * bounds of the configuration space.
         *
         * @param baseTend [in] Desired base to end transformation \f$
         * \robabx{}{desired}{\mathbf{T}}\f$
         *
         * @return List of valid solutions. Notice that the list may be empty.
         */
        virtual std::vector<math::Q> solve(rw::math::Transform3D<>& baseTend) const = 0;

        /**
           @brief Closed-form IK solver for a device.

           The device must be a serial device with 6 revolute joints described
           by DH parameters.

           The IK solver is currently implemented in terms of PieperSolver. See
           the documentation of PieperSolver for the specific requirements for
           the DH parameters.

           An exception is thrown if closed-form IK for the device is not
           supported, except that all such cases are currently not discovered.
           You should check for yourself that the closed-form IK for the device
           is correct.
        */
        static
        ClosedFormIKPtr make(const rw::models::Device& device,
                             const rw::kinematics::State& state);

        /**
           @brief Destructor
        */
        virtual ~ClosedFormIK() {}

    protected:
        /**
           @brief Constructor
        */
        ClosedFormIK() {}

    private:
        ClosedFormIK(const ClosedFormIK&);
        ClosedFormIK& operator=(const ClosedFormIK&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
