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

#ifndef rwlibs_pathplanners_sbl_SBLExpand_HPP
#define rwlibs_pathplanners_sbl_SBLExpand_HPP

/**
   @file SBLExpand.hpp
*/

#include <rw/pathplanning/QConstraint.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/DeviceJacobian.hpp>
#include <rw/models/Device.hpp>
#include <rw/kinematics/State.hpp>

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
            rw::pathplanning::QConstraintPtr constraint,
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
            rw::pathplanning::QConstraintPtr constraint,
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
            rw::pathplanning::QConstraintPtr constraint,
            rw::models::DevicePtr device,
            const rw::kinematics::State& state,
            rw::models::DeviceJacobianPtr jacobian,
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
