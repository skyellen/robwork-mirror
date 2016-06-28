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


#ifndef RW_MODELS_CARTESIAN6DOFDEVICE_HPP
#define RW_MODELS_CARTESIAN6DOFDEVICE_HPP

/**
 * @file SE3Device.hpp
 */

#include "Device.hpp"

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/MovableFrame.hpp>

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief A Cartesian 6-Dof device
     *
     * The SE3Device is a 6-dof device with 6 independent inputs that
     * enables the device to place its end-effector anywhere in the workspace.
     *
     * The @f$ \mathbf{q}\in \mathbb{R}^6 @f$ input vector maps directly to the
     * end-effector pose @f$ \robabx{b}{e}{\mathbf{x}} @f$, thus:
     *
     * @f[ \robabx{b}{e}{\mathbf{x}} =
     * \left[
     * \begin{array}{c}
     * x\\
     * y\\
     * z\\
     * \theta k_x\\
     * \theta k_y\\
     * \theta k_z
     * \end{array}
     * \right] =
     * \left[
     * \begin{array}{c}
     * q_1\\
     * q_2\\
     * q_3\\
     * q_4\\
     * q_5\\
     * q_6
     * \end{array}
     * \right] =
     * \mathbf{q} @f]
     *
     * It is easily seen that the jacobian @f$
     * {^b_6}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) = \frac{\partial
     * ^b\mathbf{x}_6}{\partial \mathbf{q}} @f$ equals the @f$ 6\times 6 @f$
     * identity matrix @f$ \mathbf{I}^{6\times 6} @f$
     *
     * The device can be seen as a "perfect" robot, it has no singularities
     * anywhere in the task space, no kinematic or dynamic limits (it can
     * instantaneous move anywhere at any time). The device is interesting in
     * simulations where performance and stability of closed-loop control
     * systems (for instance visual-servoing systems) must be evaluated - if a
     * closed-loop control system does not perform well with a "perfect" robot
     * it will probably not perform well with a real robot either.
     */
    class SE3Device : public Device {
    public:

        /**
         * @brief Constructor
         *
         * @param name [in] device name
         */
        SE3Device(const std::string& name, rw::kinematics::Frame* base, rw::kinematics::MovableFrame* mframe);


        virtual ~SE3Device(){}

        /**
         * @copydoc Device::setQ
         *
         * @pre q.size() == 6
         */
        void setQ(const math::Q& q, kinematics::State& state) const;

        /**
         * @copydoc Device::getQ
         */
        math::Q getQ(const kinematics::State& state) const;

        /**
         * @copydoc Device::getBounds
         *
         * Since the SE3Device robot is unconstrained and can move anywhere
         * whithin the taskspace each of the 6 input's are unbounded (@f$
         * [-\inf, \inf] @f$) in practice the inputs are limited to the
         * numerical limits of the real datatype, thus this method returns the
         * range ([DBL_MIN, DBL_MAX]) for each of the 6 inputs
         */
        std::pair<math::Q, math::Q> getBounds() const;


        kinematics::Frame* getBase(){ return _base; }
        const kinematics::Frame* getBase() const { return _base; }
        kinematics::Frame* getEnd(){ return _mframe; }
        const kinematics::Frame* getEnd() const { return _mframe; }

        /**
         * @brief Calculates the jacobian matrix of the end-effector described
         * in the robot base frame @f$ ^b_e\mathbf{J}_{\mathbf{q}}(\mathbf{q})
         * @f$
         *
         * @return the @f$ 6*ndof @f$ jacobian matrix: @f$
         * {^b_e}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) @f$
         *
         * Where:
         *
         * \f[
         *  {^b_n}\mathbf{J}_{\mathbf{q}}(\mathbf{q}) = \mathbf{I}^{6\times 6} =
         *  \left[
         *    \begin{array}{cccccc}
         *    1 & 0 & 0 & 0 & 0 & 0\\
         *    0 & 1 & 0 & 0 & 0 & 0\\
         *    0 & 0 & 1 & 0 & 0 & 0\\
         *    0 & 0 & 0 & 1 & 0 & 0\\
         *    0 & 0 & 0 & 0 & 1 & 0\\
         *    0 & 0 & 0 & 0 & 0 & 1\\
         *    \end{array}
         *  \right]
         * \f]
         *
         */
        math::Jacobian baseJend(const kinematics::State& state) const;


        JacobianCalculatorPtr baseJCframes(const std::vector<kinematics::Frame*>& frames, const kinematics::State& state) const{return NULL;}
        /**
         * @copydoc Device::getDOF
         *
         * This method always returns the value 6
         */
        size_t getDOF() const{
            return 6;
        }



        virtual void setBounds(const QBox& bounds);

        virtual math::Q getVelocityLimits() const;

        virtual void setVelocityLimits(const math::Q& vellimits);

        math::Q getAccelerationLimits() const;

        void setAccelerationLimits(const math::Q& acclimits);


    private:
        rw::kinematics::Frame* _base;
        rw::kinematics::MovableFrame* _mframe;
        rw::math::Q _vellimits, _acclimits;

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
