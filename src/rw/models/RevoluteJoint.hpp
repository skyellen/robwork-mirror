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

#ifndef rw_models_RevoluteJoint_HPP
#define rw_models_RevoluteJoint_HPP

/**
 * @file RevoluteJoint.hpp
 */

#include "Joint.hpp"

namespace rw { namespace kinematics {
    class State;
}} // end namespaces

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief Revolute joints.
     *
     * RevoluteJoint implements a revolute joint for the rotation about the
     * z-axis of an arbitrary displacement transform.
     */
    class RevoluteJoint : public Joint
    {
    public:
        /**
         * @brief A revolute joint with a displacement transform of \b transform.
         *
         * @param parent [in] parent frame
         * @param name [in] The name of the frame.
         * @param transform [in] The displacement transform of the joint.
         */
        RevoluteJoint(
            const std::string& name,
            const math::Transform3D<>& transform);

        /**
         * @brief The parent to frame transform for a revolute joint.
         *
         * The parent to frame transform is T * Rz(q) where:
         *
         * - T is the displacement transform of the joint;
         *
         * - q is the joint value of the joint;
         *
         * - Rz(q) is the transform that rotates a point an angle q about the
         * z-axis.
         *
         * @copydoc kinematics::Frame::getTransform
         */
        math::Transform3D<> getTransform(const kinematics::State& state) const;

        /**
           @brief The transform for a revolute joint.
        */
        static
        math::Transform3D<> getRevoluteTransform(
            const math::Transform3D<>& displacement, double q);

    private:
        math::Transform3D<> _transform;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
