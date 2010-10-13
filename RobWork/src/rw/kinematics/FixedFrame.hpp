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


#ifndef RW_KINEMATICS_FIXEDFRAME_HPP
#define RW_KINEMATICS_FIXEDFRAME_HPP

/**
 * @file FixedFrame.hpp
 */

#include "Frame.hpp"

namespace rw { namespace kinematics {

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief FixedFrame is a frame for which the transform relative to the
     * parent is constant.
     *
     * A fixed frame can for example be used for attaching a camera, say, with a
     * fixed offset relative to the tool.
     */
    class FixedFrame: public Frame
    {
    public:
        /**
         * @brief A frame fixed to its parent with a constant relative transform
         * of \b transform.
         *
         * @param name [in] The name of the frame.
         * @param transform [in] The transform with which to attach the frame.
         */
        FixedFrame(const std::string& name,
                   const math::Transform3D<>& transform);

        /**
         * @brief Sets the fixed transform of this frame.
         * @param transform [in] the new transformation of this frame
         * @notice THIS IS NOT THREAD SAFE. If you need thread safety then use
         * MovableFrame instead or make sure multiple threads are not using this
         * frame when changing the transformation.
         */
        void setTransform(const math::Transform3D<>& transform);

        /**
         * @brief get the fixed transform of this frame.
         */
        const math::Transform3D<>& getFixedTransform() const;

    private:
        void doMultiplyTransform(const math::Transform3D<>& parent,
                                 const State& state,
                                 math::Transform3D<>& result) const;

        math::Transform3D<> doGetTransform(const State& state) const;

    private:
        math::Transform3D<> _transform;
    };

    /*@}*/

}} // end namespaces

#endif // end include guard
