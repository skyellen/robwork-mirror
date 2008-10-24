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

#ifndef RW_KINEMATICS_FRAMETYPE_HPP
#define RW_KINEMATICS_FRAMETYPE_HPP

/**
 * @file FrameType.hpp
 */

#include "Frame.hpp"

#include <rw/math/Transform3D.hpp>

#include <cfloat>
#include <vector>
#include <map>
#include <string>

namespace rw { namespace kinematics {

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief Enumeration of all concrete frame types of RobWork.
     *
     * FrameType::Type is an enumeration of all frame types defined within
     * RobWork. For every implementation X of Frame, FrameType has an
     * enumeration value named FrameType::X.
     *
     * The type of a frame can be accessed via frameTypeAccessor().
     *
     * It is the responsibility of the work cell loaders to properly initialize
     * the frame type values.
     *
     * The use of FrameType is a hack introduced due to the lack of a working
     * dynamic_cast<>.
     */
    class FrameType
    {
    public:
        /**
         * @brief FrameType enumeration
         */
        enum Type {
            RevoluteJoint,
            PrismaticJoint,
            FixedFrame,
            MovableFrame,
            DependentJoint,
            Unknown
        };

        /**
         * @brief Identifier for a frame of type \b type.
         *
         * @param type [in] The type of frame.
         */
        FrameType(const Type& type) :
            _type(type)
        {}

        /**
         * @brief The frame type.
         *
         * @return The frame type.
         */
        Type get() const { return _type; }

    private:
        Type _type;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
