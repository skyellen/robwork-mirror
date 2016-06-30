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


#ifndef RWLIBS_PATHPLANNERS_SBL_SBLEXPAND_HPP
#define RWLIBS_PATHPLANNERS_SBL_SBLEXPAND_HPP

/**
   @file SBLExpand.hpp
*/

#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>

namespace rw { namespace kinematics { class State; } }
namespace rw { namespace models { class Device; } }
namespace rw { namespace models { class JacobianCalculator; } }
namespace rw { namespace pathplanning { class QConstraint; } }

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanning */
    /** @{*/

    class SBLExpand;

    //! A pointer to a SBLExpand.
    typedef rw::common::Ptr<SBLExpand> SBLExpandPtr;

    /**
       @brief Interface for sampling a configuration in the vicinity of some
       other configuration.

       SBLExpand is a primitive for planners in the SBL family. The primitive
       takes a configuration \b q as parameter and returns another configuration
       somewhere in the vicinity of \b q.

       Different implementations can have different policies with respect to
       what constraints are satisfied by the configurations returned.
    */
    class SBLExpand
    {
    public:
        /**
           @brief A configuration sampled from the vicinity of \b q.

           Implementation dependant, the sampler may return the empty
           configuration if no configurations can be sampled near \b q.
        */
        rw::math::Q expand(const rw::math::Q& q) { return doExpand(q); }

        /**
           @brief A configuration space in the shape of a box.

           The box is given by a lower and upper corner.
        */
        typedef std::pair<rw::math::Q, rw::math::Q> QBox;

        /**
           @brief Expansion within the overlap of an inner and outer box.

           Given a configuration \b q, the expand() method returns a configuration
           sampled uniformly at random from the intersection between
\code
    q + inner
\endcode
           and
\code
    outer
\endcode

           Given a \b device, you typically use \b device.getBounds() as the box
           for the outer configuration space.

           If the overlap between the boxes is empty, expand() returns the empty
           configuration.
        */
        static SBLExpandPtr makeUniformBox(
            const QBox& outer,
            const QBox& inner);

        /**
           @brief Expansion within a scaled down box of the configuration space.

           Given a configuration \b q, the expand() method samples a
           configuration uniformly at random from the intersection between
\code
    q + inner
\endcode
           and
\code
    outer
\endcode
           where \b inner equals \b outer scaled by a factor of \b ratio and
           centered at origo.

           This is a form of expansion you will use in a standard implementation
           of an SBL planner.

           \b ratio must be positive.

           If \b outer is non-empty, the expand() method will always return a
           non-empty configuration.
        */
        static SBLExpandPtr makeUniformBox(
            const QBox& outer,
            double ratio);

        /**
           @brief Sample within a box of decreasing size until a collision free
           configuration is found.

           The inner box shrinks in size as 1, 1/2, 1/3, ...

           This form of expansion is typical for SBL planners.

           The inner and outer box are specified as explained for
           makeUniformBox().
        */
        static SBLExpandPtr makeShrinkingUniformBox(
        	rw::common::Ptr<rw::pathplanning::QConstraint> constraint,
            const QBox& outer,
            const QBox& inner);

        /**
           @brief Sample within a box of shrinking size until a collision free
           configuration is found.

           The inner box shrinks in size as 1, 1/2, 1/3, ...

           This form of expansion is typical for SBL planners.

           The inner and outer box are specified as explained for
           makeUniformBox().
        */
        static SBLExpandPtr makeShrinkingUniformBox(
        	rw::common::Ptr<rw::pathplanning::QConstraint> constraint,
            const QBox& outer,
            double ratio);

        /**
           @brief Sample within a box of shrinking size until a collision free
           configuration is found.

           The size of the inner box depends on the Jacobian of the current
           configuration. The radius for the i'th dimension of the inner box is

           R_i = min(angle_max / angle_vel, disp_max / disp_vel)

           where angle_vel is the magnitude of the angular velocity and disp_vel
           is the magnitude of the translational velocity.

           If \b jacobian is NULL, a default device Jacobian is chosen based on
           \b device.

           If \b angle_max or \b disp_max is negative, a default value for the
           variable is chosen.

           The inner box shrinks in size as 1, 1/2, 1/3, ...
        */
        static SBLExpandPtr makeShrinkingUniformJacobianBox(
        	rw::common::Ptr<rw::pathplanning::QConstraint> constraint,
			rw::common::Ptr<rw::models::Device> device,
            const rw::kinematics::State& state,
            rw::common::Ptr<rw::models::JacobianCalculator> jacobian,
            double angle_max = -1,
            double disp_max = -1);

        /**
           @brief Destructor
        */
        virtual ~SBLExpand() {}

    protected:
        /**
           @brief Constructor
        */
        SBLExpand() {}

        /**
           @brief Subclass implementation of the expand() method.
        */
        virtual rw::math::Q doExpand(const rw::math::Q& q) = 0;

    private:
        SBLExpand(const SBLExpand&);
        SBLExpand& operator=(const SBLExpand&);
    };

    /* @} */
}} // end namespaces

#endif // end include guard
