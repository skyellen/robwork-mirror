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

#ifndef rw_models_PrismaticJoint_HPP
#define rw_models_PrismaticJoint_HPP

/**
 * @file PrismaticJoint.hpp
 */

#include "Joint.hpp"

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/


    /**
     * @brief Prismatic joints.
     *
     * PrismaticJoint implements a prismatic joint for the displacement in the
     * direction of the z-axis of an arbitrary displacement transform.
     */
    class PrismaticJoint : public Joint
    {
    public:
        /**
         * @brief A prismatic joint with a displacement transform of \a
         * transform.
         *
         * @param parent [in] the parent frame
         * @param transform [in] The displacement transform of the joint.
         * @param name [in] The name of the frame.
         */
        PrismaticJoint(
            const std::string& name,
            const math::Transform3D<>& transform);

        /**
         * @brief The parent to frame transform for a prismatic joint.
         *
         * The parent to frame transform is T * Tz(q) where:
         *
         * - T is the displacement transform of the joint;
         *
         * - q is the joint value of the joint;
         *
         * - Tz(q) is the transform that translates a point an distance q in the
         * direction of the z-axis.
         *
         * @copydoc kinematics::Frame::getTransform
         */
        math::Transform3D<> getTransform(const kinematics::State& state) const;

        /// @cond SHOW_ALL
        /**
           @brief The transform for a prismatic joint.
         */
        static
        math::Transform3D<> getPrismaticTransform(
            const math::Transform3D<>& displacement, double q);
        /// @endcond

        /// @cond SHOW_ALL
        /**
           @brief The transform of the joint for a given joint value.
        */
        void getJointValueTransform(
            const math::Transform3D<>& parent,
            double q,
            math::Transform3D<>& result) const;
        /// @endcond

    private:
        void doGetTransform(
            const math::Transform3D<>& parent,
            const kinematics::State& state,
            math::Transform3D<>& result) const;

    private:
        math::Transform3D<> _transform;
    };
    /*@}*/
}} // end namespaces

#endif // end include guard
