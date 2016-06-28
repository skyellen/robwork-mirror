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


#ifndef RW_KINEMATICS_FRAMETYPE_HPP
#define RW_KINEMATICS_FRAMETYPE_HPP

/**
 * @file FrameType.hpp
 */

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
