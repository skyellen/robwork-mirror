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

#ifndef RW_MODELS_JACOBIANUTIL_HPP
#define RW_MODELS_JACOBIANUTIL_HPP

/**
 * @file JacobianUtil.hpp
 */

#include <rw/math/Transform3D.hpp>

namespace rw { namespace math {
    class Jacobian;
}}

namespace rw { namespace kinematics {
    class State;
    class Frame;
    class FKTable;
}}

namespace rw { namespace models {

    class BasicDevice;
    class PassiveRevoluteFrame;

    /** @addtogroup models */
    /*@{*/

    /**
       @brief Primitive utilities for computing jacobians for joints of various
       types.
    */
    class JacobianUtil
    {
    public:
        /**
           @brief Add to column \b col of \b jacobian the Jacobian of a revolute
           joint with transform \b joint for a tool position of \b tcp.

           The Jacobian is given relative to the common world frame of \b joint
           and \b tcp.
        */
        static void addRevoluteJacobianCol(
            math::Jacobian& jacobian,
            int row,
            int col,
            const math::Transform3D<>& joint,
            const math::Transform3D<>& tcp);

        /**
           @brief Add to column \b col of \b jacobian the Jacobian of a
           prismatic joint with transform \b joint for a tool position of \b
           tcp.

           The Jacobian is given relative to the common world frame of \b joint
           and \b tcp.
        */
        static void addPrismaticJacobianCol(
            math::Jacobian& jacobian,
            int row,
            int col,
            const math::Transform3D<>& joint,
            const math::Transform3D<>& tcp);

        /**
           @brief Add to column \b col of \b jacobian the Jacobian for a passive
           revolute joint at position \b passive that controls the tool at
           position \b tcp. The joint scaling factor of the passive joint is \b
           scale.

           The Jacobian is given relative to the common world frame of \b joint
           and \b tcp.
        */
        static void addPassiveRevoluteJacobianCol(
            math::Jacobian& jacobian,
            int row,
            int col,
            const math::Transform3D<>& passive,
            const math::Transform3D<>& tcp,
            double scale);

        /**
           @brief True iff \b child is in the subtree of \b parent for a tree
           structure of \b state.

           <code>isInSubTree(frame, frame, state)</code> is true always.

           This utility function is used for checking if a given joint does
           affect some tcp frame or if the Jacobian column for that joint should
           be considered zero.

           isInSubTree() runs in time proportional to the size of the subtree.
        */
        static bool isInSubTree(
            const kinematics::Frame& parent,
            const kinematics::Frame& child,
            const kinematics::State& state);

        /**
           @brief True iff \b child is controlled by a joint of \b device for a
           tree structure of \b state.
        */
        static bool isControlledBy(
            const BasicDevice& device,
            const PassiveRevoluteFrame& child,
            const kinematics::State& state);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
