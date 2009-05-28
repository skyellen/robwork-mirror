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

#ifndef RW_INVKIN_SIMPLEMULTISOLVER_HPP
#define RW_INVKIN_SIMPLEMULTISOLVER_HPP

/**
 * @file SimpleMultiSolver.hpp
 */

#include <rw/invkin/IterativeMultiIK.hpp>
#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/kinematics/FKRange.hpp>

#include <rw/models/JacobianCalculator.hpp>

#include <vector>

namespace rw { namespace models {
    class JointDevice;
    class SerialDevice;
    class TreeDevice;
    class Device;
}} // end namespaces

namespace rw { namespace invkin {

    /** \addtogroup invkin */
    /*@{*/

    /**
     * \brief Another inverse kinematics algorithm. This algorithm does not take
     * joint limits into account. Which means that solutions returned by this
     * algorithm can contain joint values that are out of bounds.
     *
     * The method uses an Newton-Raphson iterative approach
     *
     * The method is, like the ResolvedRate method, based on the following
     * approximation:
     *
     * \f$ \Delta \mathbf{x}\approx \mathbf{J}(\mathbf{q})\Delta \mathbf{q} \f$
     *
     * In this method, however, \f$ \Delta \mathbf{x} \f$ is calculated in a
     * different way:
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
    class SimpleMultiSolver : public IterativeMultiIK
    {
    public:
        /**
         * @brief Constructs SimpleMultiSolver for TreeDevice. Uses the default
         * end effectors of the treedevice
         */
        SimpleMultiSolver(const models::TreeDevice* device,
                          const kinematics::State& state);

        /**
         * @brief Constructs SimpleMultiSolver for a
         * JointDevice(SerialDevice and TreeDevice). It does not use
         * the default end effectors. A list of interest frames are
         * given instead.
         */
        SimpleMultiSolver(const models::JointDevice* device,
                          const std::vector<kinematics::Frame*>& foi,
                          const kinematics::State& state);

        /**
         * @brief configures the iterative solver to return the best fit
         * found, even though error criterias was not met.
         * @param returnBestFit [in] set to true if you want best fit returned.
         */
        void setReturnBestFit(bool returnBestFit){
            _returnBestFit = returnBestFit;
        }

        /**
         * @copydoc IterativeIK::solve
         */
        std::vector<math::Q> solve(const std::vector<math::Transform3D<> >& baseTend,
                                   const kinematics::State& state) const;

        /**
         * @brief performs a local search toward the the target bTed. No via points
         * are generated to support the convergence and robustness.
         * @param bTed [in] the target end pose
         * @param maxError [in] the maximal allowed error
         * @param state [in/out] the starting position for the search. The end position will
         * also be saved in this state.
         * @param maxIter [in] max number of iterations
         * @param untilSmallChange [in] if true the error difference between two
         * succesive iterations need to be smaller than \b maxError. If false then the
         * error of a iteration need to be smaller than \b maxError.
         * @return true if error is below max error
         * @note the result will be saved in state
         */
        bool solveLocal(const std::vector<rw::math::Transform3D<> > &bTed,
                        std::vector<double>& maxError,
                        kinematics::State &state,
                        int maxIter,
                        bool untilSmallChange=false) const;


        /**
         * @brief sets the maximal step length that is allowed on the
         * local search towards the solution.
         * @param qlength [in] maximal step length in quaternion
         * @param plength [in] maximal step length in position
         */
        void setMaxLocalStep(double qlength, double plength);

    private:
        const models::Device* _device;
        rw::common::Ptr<models::JacobianCalculator> _jacCalc;
        std::vector<kinematics::Frame*> _foi; // frames of interest, end frames
        std::vector<boost::shared_ptr<kinematics::FKRange> > _fkranges;
        double _maxQuatStep;
        bool _returnBestFit;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
