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


#ifndef RWLIBS_ALGORITHMS_QPCONTROLLER_QPCONTROLLER_HPP
#define RWLIBS_ALGORITHMS_QPCONTROLLER_QPCONTROLLER_HPP

/**
 * @file QPController.hpp
 */

#include <cmath>
#include <boost/numeric/ublas/vector.hpp>

#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>

namespace rwlibs { namespace algorithms {

    /** @addtogroup algorithms */
    /* @{*/

    /**
     * The QPController provides an visual servoing control scheme, which is based on
     * quadratic optimization. The method computes the \f$\mathbf{\dot{q}}\f$ minimizing
     * \f$\|\mathbf{J}\mathbf{\dot{q}}-\mathbf{\dot{x}}\|^2\f$, subject to the joint position,
     * velocity and acceleration limits.
     *
     * See the paper: L.-P. Ellekilde, P. Favrhold, M. Paulin and H.G. Petersen, "Robust
     * Control for High-Speed Visual Servoing Applications", To appear in International
     * Journal for Advanced Robotic Systems, vol. 4, no. 3, 2007.
     */
    class QPController {
    public:
        /**
         * @brief Construct QPController object
         * @param h [in] step-size to use
         * @param state [in] state of the workcell
         * @param device [in] device to control
         */
        QPController(double h,
                     const rw::kinematics::State& state,
                     rw::models::Device* device);

        /**
         * @brief destructor
         */
        virtual ~QPController();

        /**
         * @brief Computes joint velocities for a tool velocity
         *
         * The method provides the, in a least square sense, optimal joint
         * velocity for the specified tool velocity screw. That is the
         * \f$\mathbf{\dot{q}}\f$ minimizing \f$\|\mathbf{J}\mathbf{\dot{q}}-\mathbf{\dot{x}}\|^2\f$,
         * subject to the joint position, velocity and acceleration limits.
         *
         * @param q [in] current device joint configuration
         * @param dq [in] current device joint velocity
         * @param tcpscrew [in] desired velocity screw of tcp
         *
         * @return the optimal joint velocity
         */
        rw::math::Q solve(const rw::math::Q& q,
                          const rw::math::Q& dq,
                          const rw::math::VelocityScrew6D<>& tcpscrew);

    private:
        typedef rw::math::Q::BoostVector VectorBase;

        VectorBase inequalitySolve(const boost::numeric::ublas::matrix<double>& G,
        				      const VectorBase& b,
        				      const VectorBase& lower,
        				      const VectorBase& upper);

        void calculateVelocityLimits(VectorBase& lower,
        							 VectorBase& upper,
        							 const VectorBase& q,
        							 const VectorBase& dq);

        double _h;
        size_t _n;
        rw::models::Device* _device;
        rw::kinematics::State _state;

        VectorBase _qmin;
        VectorBase _qmax;

        VectorBase _vmin;
        VectorBase _vmax;

        VectorBase _amin;
        VectorBase _amax;

        //Stuff for stats
        char* _lowerLimitType;
        char* _upperLimitType;
        char* _statusArray;
        int* _accLimit;
        int* _velLimit;
        int* _posLimit;
    };

    /**@}*/

}} // end namespaces

#endif // end include guard
