/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
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

#ifndef rw_models_Cartesian6DOFDevice_HPP
#define rw_models_Cartesian6DOFDevice_HPP

/**
 * @file Cartesian6DOFDevice.hpp
 */

#include "Device.hpp"

#include <rw/math/Transform3D.hpp>

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief A Cartesian 6-Dof device
     *
     * The Cartesian6DOFDevice is a 6-dof device with 6 independent inputs that
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
    class Cartesian6DOFDevice : public Device {
    public:

        /**
         * @brief Constructor
         *
         * @param name [in] device name
         */
        Cartesian6DOFDevice(const std::string& name);

        /**
         * @copydoc Device::setQ
         *
         * @pre q.size() == 6
         */
        void setQinState(const math::Q& q, kinematics::State& state) const;

        /**
         * @copydoc Device::getQ
         */
        math::Q getQfromState(const kinematics::State& state) const;

        /**
         * @copydoc Device::getBounds
         *
         * Since the freeflying robot is unconstrained and can move anywhere
         * whithin the taskspace each of the 6 input's are unbounded (@f$
         * [-\inf, \inf] @f$) in practice the inputs are limited to the
         * numerical limits of the real datatype, thus this method returns the
         * range ([DBL_MIN, DBL_MAX]) for each of the 6 inputs
         */
        std::pair<math::Q, math::Q> getBounds() const;

        /**
         * @brief Calculates the homogeneous transform from base to end-effector
         * @f$ \robabx{b}{e}{\mathbf{T}} @f$.
         *
         * @return the homogeneous transform @f$ \robabx{b}{e}{\mathbf{T}} @f$
         */
        math::Transform3D<double> bTe();

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
        math::Jacobian bJe();

        /**
         * @copydoc Device::getDOF
         *
         * This method always returns the value 6
         */
        size_t getDOF() const{
            return 6;
        }

    private:
        math::Q _q;
        math::Transform3D<> _transform;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
