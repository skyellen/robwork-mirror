/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

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

#ifndef RW_INVKIN_SIMPLESOLVER_HPP
#define RW_INVKIN_SIMPLESOLVER_HPP

/**
 * @file SimpleSolver.hpp
 */

#include "IterativeIK.hpp"
#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/kinematics/FKRange.hpp>
#include <rw/models/JacobianCalculator.hpp>
#include <vector>

namespace rw { namespace models {
    class Device;
}} // end namespaces

namespace rw { namespace invkin {

    /** \addtogroup invkin */
    /*@{*/

    /**
     * \brief Another inverse kinematics algorithm. This algorithm does not take
     * joint limits into account. Which means that solutions returned by this algorithm
     * can contain joint values that are out of bounds.
     *
     * The method uses an Newton-Raphson iterative approach
     *
     * The method is, like the ResolvedRate method, based on the following approximation:
     *
     * \f$ \Delta \mathbf{x}\approx \mathbf{J}(\mathbf{q})\Delta \mathbf{q} \f$
     *
     * In this method, however, \f$ \Delta \mathbf{x} \f$ is calculated in a different way:
     *
     * \f$
     * \robabx{b}{e}{\mathbf{T}}(\mathbf{q}) =
     * \robabx{b}{e}{\mathbf{T}}(\mathbf{q}_{init}+\Delta \mathbf{q}) \approx
     * \robabx{b}{e}{\mathbf{T}}(\mathbf{q}_{init})\Delta \mathbf{T}(\Delta \mathbf{q}) =
     * \robabx{b}{e*}{\mathbf{T}}
     * \f$
     *
     * \f$
     * \Delta \mathbf{T}(\Delta \mathbf{q}) = \robabx{j}{i}{\mathbf{T}}\robabx{b}{e*}{\mathbf{T}}=\mathbf{I}^{4x4}+\mathbf{L}
     * \f$
     *
     * \f$
     * \mathbf{L} =
     * \left[
     * \begin{array}{cccc}
     *         0 & -\omega_z &  \omega_y & v_x \\
     *  \omega_z &         0 & -\omega_x & v_y \\
     * -\omega_y &  \omega_x &         0 & v_z \\
     *         0 &         0 &         0 &   0
     * \end{array}
     * \right]
     * \f$
     */
    class SimpleSolver : public IterativeIK
    {
    public:
        /**
         * @brief Constructs SimpleSolver for device
         */
        SimpleSolver(models::Device* device, const kinematics::State& state);

        /**
         * @brief Constructs SimpleSolver for device
         */
        SimpleSolver(models::Device* device,
                     rw::kinematics::Frame *foi,
                     const kinematics::State& state);

        /**
         * @copydoc IterativeIK::solve
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

    private:
        const models::Device* _device;
        double _maxQuatStep;
        kinematics::FKRange _fkrange;
        rw::common::Ptr<models::JacobianCalculator> _devJac;

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
