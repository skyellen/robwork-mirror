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

#ifndef rw_models_JointTransform_HPP
#define rw_models_JointTransform_HPP

/**
 * @file JointTransform.hpp
 */

#include <rw/math/Transform3D.hpp>

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
       @brief Joint transforms of various joints.
     */
    class JointTransform
    {
    public:
        /**
           @brief The transform for a revolute joint.
        */
        static
        math::Transform3D<> getRevoluteTransform(
            const math::Transform3D<>& displacement, double q);

        /**
           @brief The transform for a prismatic joint.
         */
        static
        math::Transform3D<> getPrismaticTransform(
            const math::Transform3D<>& displacement, double q);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
