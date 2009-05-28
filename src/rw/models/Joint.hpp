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


        virtual void getJacobian(size_t row, size_t col, const math::Transform3D<>& joint, const math::Transform3D<>& tcp, math::Jacobian& jacobian) const = 0;

        /*math::Transform3D<> getJointTransform(const math::Q& q) const {
            return doGetJointTransform(q);
        }

        void multiplyJointTransform(const math::Transform3D<>& parent,
                                    const math::Q& q,
                                    math::Transform3D<>& result) const {
            doMultiplyJointTransform(parent, q, result);
        }*/



    protected:
        /*virtual math::Transform3D<> doGetJointTransform(const math::Q& q) const = 0;


        virtual void doMultiplyJointTransform(const math::Transform3D<>& parent,
                                              const math::Q& q,
                                              math::Transform3D<>& result) const = 0;
*/
        /**
         * @copydoc Frame::doGetTransform
         */
       /* math::Transform3D<> doGetTransform(const kinematics::State& state) const {
            return doGetJointTransform(math::Q(getDOF(), getQ(state)));
        }
*/
        /**
         * @copydoc Frame::doMultiplyTransform
         */
  /*      void doMultiplyTransform(const math::Transform3D<>& parent,
                                 const kinematics::State& state,
                                 math::Transform3D<>& result) const {
            doMultiplyJointTransform(parent, math::Q(getDOF(), getQ(state)), result);
        }
*/


    private:
        std::pair<math::Q, math::Q> _bounds;
        math::Q _maxVelocity;
        math::Q _maxAcceleration;

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
