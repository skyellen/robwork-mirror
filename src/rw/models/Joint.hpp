/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RW_MODELS_JOINT_HPP
#define RW_MODELS_JOINT_HPP

/**
 * @file Joint.hpp
 */

#include <rw/kinematics/Frame.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Jacobian.hpp>

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief A Joint is a Frame with assignable values for
     * position, velocity and acceleration limits.
     *
     */
    class Joint : public kinematics::Frame
    {
    public:
        /**
         * @brief Default constructor for the joint interface.
         *
         * @param name [in] The name of the frame.
         * @param dof [in] the degrees of freedom of this joint
         */

        Joint(const std::string& name, size_t dof);

        /**
         * @brief Virtual destructor
         */
        virtual ~Joint() {}

        /**
         * @brief Sets joint bounds
         * @param bounds [in] the lower and upper bounds of this joint
         */
        void setBounds(const std::pair<const math::Q, const math::Q>& bounds){ _bounds = bounds; }

        /**
         * @brief Gets joint bounds
         * @return the lower and upper bound of this joint
         */
        const std::pair<math::Q, math::Q>& getBounds() const { return _bounds; }

        /**
         * @brief Sets max velocity of joint
         * @param maxVelocity [in] the new maximum velocity of the joint
         */
        void setMaxVelocity(const math::Q& maxVelocity)
        { _maxVelocity = maxVelocity; }

        /**
         * @brief Gets max velocity of joint
         * @return the maximum velocity of the joint
         */
        const math::Q& getMaxVelocity() const
        { return _maxVelocity; }

        /**
         * @brief Sets max acceleration of joint
         * @param maxAcceleration [in] the new maximum acceleration of the joint
         */
        void setMaxAcceleration(const math::Q& maxAcceleration)
        { _maxAcceleration = maxAcceleration; }

        /**
         * @brief Gets max acceleration of joint
         * @return the maximum acceleration of the joint
         */
        const math::Q& getMaxAcceleration() const
        { return _maxAcceleration; }


        /**
         * @brief Finds the Jacobian of the joints and adds it in \b jacobian.
         *
         * Calculates the Jacobian contribution to the device Jacobian when controlling a frame \b tcp and given a
         * current joint pose \b joint.
         *
         * The values are stored from row \b row to \b row+5 and column \b col to col+(joint.getDOF()-1).
         *
         * @param row [in] Row where values should be stored
         * @param col [in] Column where values should be stored
         * @param joint [in] Transform of the joint
         * @param tcp [in] Transformation of the point to control
         * @param jacobian [in] Jacobian to which to add the results.
         */
        virtual void getJacobian(size_t row, size_t col, const math::Transform3D<>& joint, const math::Transform3D<>& tcp, math::Jacobian& jacobian) const = 0;

    private:
        std::pair<math::Q, math::Q> _bounds;
        math::Q _maxVelocity;
        math::Q _maxAcceleration;

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
