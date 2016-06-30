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


#ifndef RWLIBS_ALGORITHMS_QPCONTROLLER_IKQPSOLVER_HPP
#define RWLIBS_ALGORITHMS_QPCONTROLLER_IKQPSOLVER_HPP

/**
 * @file IKQPSolver.hpp
 */

#include "QPController.hpp"

#include <rw/invkin/IterativeIK.hpp>
#include <rw/kinematics/State.hpp>

namespace rw { namespace models {
    class SerialDevice;
}} // end namespaces

namespace rwlibs { namespace algorithms {

    /** \addtogroup algorithms */
    /*@{*/

    /**
     * @brief Iterative inverse kinematics solved based on the QPController
     *
     * The IKQPSolver works very similar to the SimpleSolver. However, instead
     * of using a simple inverse Jacobian it uses the rw::algorithms::QPController
     * making it robust to singularities and ensuring joint limits.
     *
     * Usually the IKQPSolver runs order of magnitudes slower than the standard
     * SimpleSolver. Solutions returned by the IKQPSolver might not be exact
     * solutions, but are least square fits. Given a target outside the robot workspace
     * the IKQPSolver will, given enough iterations, return the least square solution.
     *
     * Notice that the IKQPSolver is a local method. It there is thus no guarantee that
     * the solution returned will be the global least square.
     */
    class IKQPSolver: public rw::invkin::IterativeIK {
    public:
        /**
         * @brief Constructs IKQPSolver for device
         *
         * It is required that the device has correct joint position, velocity
         * and acceleration limits.
         *
         * @param device [in] Device to solve for         * @param state [in] State of the workcell
         */
        IKQPSolver(rw::models::SerialDevice* device,
                   const rw::kinematics::State& state);

        /**
         * @copydoc rw::invkin::IterativeIK::solve
         */
        std::vector<rw::math::Q> solve(const rw::math::Transform3D<>& baseTend,
                                       const rw::kinematics::State& state) const;

        void setCheckJointLimits(bool limit){};
    private:
        QPController _qpcontroller;
        const rw::models::SerialDevice* _device;
        rw::common::PropertyMap _properties;
        rw::kinematics::State _state;
        double _maxQuatStep;

        bool performLocalSearch(const rw::models::SerialDevice *device,
                       			const rw::math::Transform3D<> &bTed,
                       			double maxError,
                       			rw::kinematics::State &state,
                       			unsigned int maxIter) const;
    };

}} // end namespaces

#endif // end include guard
