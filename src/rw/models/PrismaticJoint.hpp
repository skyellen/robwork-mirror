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
           @brief A prismatic joint with a displacement transform of \b
           transform.

           @param name [in] The name of the frame.
           @param transform [in] The displacement transform of the joint.
        */
        static
        PrismaticJoint* make(
            const std::string& name,
			const math::Transform3D<>& transform);

        /// @cond SHOW_ALL
        /**
           @brief The transform of the joint for a given joint value.
        */
        void getJointValueTransform(
            const math::Transform3D<>& parent,
            double q,
            math::Transform3D<>& result) const;
        /// @endcond

    protected:
        /**
           @brief Subclasses should call this constructor.
        */
        explicit PrismaticJoint(const std::string& name) :
            Joint(name)
        {}

    private:
        void doGetTransform(
            const math::Transform3D<>& parent,
            const kinematics::State& state,
            math::Transform3D<>& result) const;

        virtual void doGetJointValueTransform(
            const math::Transform3D<>& parent,
            double q,
            math::Transform3D<>& result) const = 0;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
