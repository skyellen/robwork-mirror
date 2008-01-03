/*********************************************************************
 * RobWork Version 0.2
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

#ifndef rw_iksolvers_ResolvedRateSolver_HPP
#define rw_iksolvers_ResolvedRateSolver_HPP

/**
 * @file ResolvedRateSolver.hpp
 */

#include "IterativeIK.hpp"

#include <rw/models/Device.hpp>
#include <rw/common/PropertyMap.hpp>

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
         * @brief Constructs ResolvedRateSolver for device
         */
        ResolvedRateSolver(const models::Device* device);

        /**
         * @copydoc rw::inversekinematics::IterativeIK::solve
         */
        std::vector<math::Q> solve(
            const math::Transform3D<>& baseTend,
            const kinematics::State& state) const; 

        /**
         * @brief sets the maximal step length that is allowed on the
         * local search towards the solution. 
         * @param qlength [in] maximal step length in quaternion
         * @param plength [in] maximal step length in position 
         */
        void setMaxLocalStep(double qlength, double plength);
        
    private:
        const models::Device* _device;
        double _maxQuatStep;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
