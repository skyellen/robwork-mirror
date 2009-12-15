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


#ifndef RW_INVKIN_RESOLVEDRATESOLVER_HPP
#define RW_INVKIN_RESOLVEDRATESOLVER_HPP

/**
 * @file ResolvedRateSolver.hpp
 */

#include "IterativeIK.hpp"

#include <rw/models/Device.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/kinematics/FKRange.hpp>
#include <rw/models/JacobianCalculator.hpp>

namespace rw { namespace kinematics {
    class State;
}} // end namespaces

namespace rw { namespace invkin {

    /** @addtogroup invkin */
    /*@{*/

    /**
     * \brief This inverse kinematics method uses the "inverse jacobian method"
     * aka. "resolved-rate motion control" to perform inverse kinematics.
     * This algorithm does not take
     * joint limits into account. Which means that solutions returned by this algorithm
     * can contain joint values that are out of bounds.
     *
     * The method uses an Newton-Raphson iterative approach
     *
     * The method is based on the following approximation:
     *
     * \f$ \Delta \mathbf{x}\approx \mathbf{J}(\mathbf{q})\Delta \mathbf{q} \f$
     *
     * We can calculate \f$ \Delta \mathbf{x} \f$ of frame \f$ e \f$ wrt. base frame as:
     *
     * \f$ \robabcx{b}{e}{e*}{\Delta \mathbf{d}}=\robabx{b}{e*}{\mathbf{d}}-\robabx{b}{e}{\mathbf{d}} \f$
     *
     * \f$ \robabcx{b}{e}{e*}{\Delta \thetak}=\robabx{b}{e}{\mathbf{R}}\robabcx{e}{e}{e*}{\Delta \thetak} \f$
     *
     *
     * We can calculate \f$ \Delta \mathbf{x} \f$ of frame \f$ e \f$ wrt. frame \f$ e \f$ as:
     *
     * \f$ \robabcx{e}{e}{e*}{\Delta \mathbf{d}}=pos(\robabx{e}{e*}{\mathbf{T}}) \f$
     *
     * \f$ \robabcx{e}{e}{e*}{\Delta \thetak}=aa(\robabx{e}{e*}{\mathbf{T}}) \f$
     *
     * The resulting equation is thus:
     *
     * \f$ \robabcx{b}{e}{e*}{\Delta \mathbf{x}}=\robabcdx{b}{e}{q}{q}{\mathbf{J}}(\mathbf{q})\Delta\mathbf{q} \f$
     *
     * or
     *
     * \f$ \robabcx{e}{e}{e*}{\Delta \mathbf{x}}=\robabcdx{e}{e}{q}{q}{\mathbf{J}}(\mathbf{q})\Delta\mathbf{q} \f$
     *
     * The equation is solved by using the Moore-penrose inverse of \f$ \mathbf{J} \f$
     *
     * \f$ \Delta \mathbf{q}\approx \mathbf{J}^+(\mathbf{q})\Delta \mathbf{x} \f$
     *
     */
    class ResolvedRateSolver : public IterativeIK
    {
    public:
        /**
         *  @brief ResolvedRateSolver for a device and state
         *
         *  Solves the inverse kinematics problem from base to end of device
         *
         *  @param device [in] Device to solve for
         *  @param state [in] State giving the relevant workcell setup
         */
        ResolvedRateSolver(models::DevicePtr device, const kinematics::State& state);


        /**
         * @brief ResolvedRateSolver for performing inverse kinematics for a specific frame
         *
         * Solves the invese kinematics problem from base of device to \b end.
         *
         * @param device [in] Device to perform invese kinematics for
         * @param end [in] Frame to solve for
         * @param state [in] State giving the relevant workcell setup
         */
        ResolvedRateSolver(models::DevicePtr device, kinematics::Frame *end, const kinematics::State& state);


        /**
         * @copydoc IterativeIK::solve
         *
         * @note interpolates the distance from current end pose to the
         * given end pose. This enables fast and robust inverse kinematic search
         */
        std::vector<math::Q> solve(const math::Transform3D<>& baseTend,
                                   const kinematics::State& state) const;

        /**
         * @brief sets the maximal step length that is allowed on the
         * local search towards the solution.
         * @param qlength [in] maximal step length in quaternion
         * @param plength [in] maximal step length in position
         */
        void setMaxLocalStep(double qlength, double plength);

        /**
         * @brief performs a local search toward the the target bTed. No via points
         * are generated to support the convergence and robustness.
         * @param bTed [in] the target end pose
         * @param maxError [in] the maximal allowed error
         * @param state [in/out] the starting position for the search. The end position will
         * also be saved in this state.
         * @param maxIter [in] max number of iterations
         * @return true if error is below max error
         * @note the result will be saved in state
         */
        bool solveLocal(const math::Transform3D<> &bTed,
                        double maxError,
                        kinematics::State &state,
                        int maxIter) const;

        /**
         * @copydoc InvKinSolver::setCheckJointLimits
         */
        void setCheckJointLimits(bool check);

    private:
        models::DevicePtr _device;
        double _maxQuatStep;
        kinematics::FKRange _fkrange;
        rw::common::Ptr<models::JacobianCalculator> _devJac;
        bool _checkForLimits;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
