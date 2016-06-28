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


#ifndef RW_INVKIN_JacobianIKSolverM_HPP
#define RW_INVKIN_JacobianIKSolverM_HPP

/**
 * @file JacobianIKSolverM.hpp
 */

#include <rw/invkin/IterativeMultiIK.hpp>
#include <rw/math/Q.hpp>

#include <vector>

namespace rw { namespace kinematics { class Frame; } }
namespace rw { namespace kinematics { class FKRange; } }
namespace rw { namespace kinematics { class State; } }

namespace rw { namespace models {
    class JointDevice;
    class TreeDevice;
    class Device;
    class JacobianCalculator;
}} // end namespaces

namespace rw { namespace invkin {

    /** \addtogroup invkin */
    /*@{*/

    /**
     * \brief A Jacobian based iterative inverse kinematics algorithm for devices with
     * multiple end effectors.
     *
     * This algorithm does not implicitly handle joint limits,
     * however it is possible to force the solution within joint
     * limits using clamping in each iterative step. If joint clamping is not enabled then this
     * algorithm might contain joint values that are out of bounds.
     *
     * The method uses an Newton-Raphson iterative approach and is based on using the inverse of
     * the device Jacobian to compute each local solution in each iteration. Several methods for
     * calculating/approximating the inverse Jacobian are available, where the SVD method currently is
     * the most stable, see the JacobianSolverType option for additional options.
     *
     * The method is based on the following approximation:
     *
     * \f$ \Delta \mathbf{x}\approx \mathbf{J}(\mathbf{q})\Delta \mathbf{q} \f$
     *
     * Where \f$ \Delta \mathbf{x} \f$ is calculated like:
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
    class JacobianIKSolverM : public IterativeMultiIK
    {
    public:
        //! @brief the type of Jacobian solver
        typedef enum{Transpose, SVD, DLS, SDLS} JacobianSolverType;


        /**
         * @brief Constructs JacobianIKSolverM for TreeDevice. Uses the default
         * end effectors of the TreeDevice
         */
        JacobianIKSolverM(const models::TreeDevice* device,
                          const kinematics::State& state);

        /**
         * @brief Constructs JacobianIKSolverM for a
         * JointDevice(SerialDevice and TreeDevice). It does not use
         * the default end effectors. A list of interest frames are
         * given instead.
         */
        JacobianIKSolverM(const models::JointDevice* device,
                          const std::vector<kinematics::Frame*>& foi,
                          const kinematics::State& state);

        //! @brief destructor
        virtual ~JacobianIKSolverM(){}

        /**
         * @brief configures the iterative solver to return the best fit
         * found, even though error criteria was not met.
         * @param returnBestFit [in] set to true if you want best fit returned.
         */
        void setReturnBestFit(bool returnBestFit){
            _returnBestFit = returnBestFit;
        }

        /**
         * @brief enables clamping of the solution such that solution always is within joint limits.
         * @param enableClamping [in] true to enable clamping, false otherwise
         */
        void setClampToBounds(bool enableClamping){_useJointClamping=enableClamping;};

        /**
         * @brief the solver may fail or be very slow if the the solution is too far from the
         * initial configuration. This function enables the use of via points generated using
         * an interpolation from initial end effector configuration to the goal target.
         * @param enableInterpolation [in] set true to enable interpolation, false otherwise
         */
        void setEnableInterpolation(bool enableInterpolation){ _useInterpolation = enableInterpolation; };

        /**
         * @brief set the type of solver to use for stepping toward a solution
         * @param type [in] the type of Jacobian solver
         */
        void setSolverType(JacobianSolverType type){ _solverType = type; };

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
         * successive iterations need to be smaller than \b maxError. If false then the
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
        double _interpolationStep;
        bool _returnBestFit;
        bool _useJointClamping, _useInterpolation;
        JacobianSolverType _solverType;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
