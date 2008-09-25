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

#ifndef rw_models_FixedJoint_HPP
#define rw_models_FixedJoint_HPP

/**
 * @file FixedJoint.hpp
 */

#include "Joint.hpp"

namespace rw { namespace kinematics {
    class State;
}} // end namespaces

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
       @brief Fixed joints.

       FixedJoint is a joint with a transform equal to the displacement
       transform provided as input. FixedJoint is the joint equivalent of
       FixedFrame.

       Fixed joints are useful when you want a single joint value of some dummy
       joint (i.e. the fixed joint) to control a number of passive joints.
     */
    class FixedJoint : public Joint
    {
    public:
        /**
         * @brief A fixed joint with a displacement transform of \b transform.
         *
         * @param parent [in] parent frame
         * @param name [in] The name of the frame.
         * @param transform [in] The displacement transform of the joint.
         */
        FixedJoint(
            const std::string& name,
            const math::Transform3D<>& transform);

        /**
           @brief The fixed displacement transform.

           @copydoc kinematics::Frame::getTransform
         */
        math::Transform3D<> getTransform(const kinematics::State& state) const
        { return _transform; }

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
