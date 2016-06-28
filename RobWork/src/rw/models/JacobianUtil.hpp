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
}}

namespace rw { namespace models {

    //class BasicDevice;
    //class DependentRevoluteJoint;

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
        static void addRevoluteJacobianCol(math::Jacobian& jacobian,
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
        static void addPrismaticJacobianCol(math::Jacobian& jacobian,
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
        static void addPassiveRevoluteJacobianCol(math::Jacobian& jacobian,
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
        static bool isInSubTree(const kinematics::Frame& parent,
                                const kinematics::Frame& child,
                                const kinematics::State& state);

        /**
           @brief True iff \b child is controlled by a joint of \b device for a
           tree structure of \b state.
        */
       /* static bool isControlledBy(const BasicDevice& device,
                                   const DependentRevoluteJoint& child,
                                   const kinematics::State& state);
                                   */
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
