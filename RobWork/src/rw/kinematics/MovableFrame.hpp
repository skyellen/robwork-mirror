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


#ifndef RW_KINEMATICS_MOVABLEFRAME_HPP
#define RW_KINEMATICS_MOVABLEFRAME_HPP

/**
 * @file MovableFrame.hpp
 */

#include "Frame.hpp"
#include "State.hpp"

namespace rw { namespace kinematics {

    /** @addtogroup kinematics */
    /* @{ */

    /**
     * @brief MovableFrame is a frame for which it is possible to freely
     * change the transform relative to the parent.
     *
     * A MovableFrame can for example be used for modelling objects moving in
     * the scene based on e.g. user input.
     */
    class MovableFrame: public Frame
    {
    public:
        /**
         * @brief Construct a MovableFrame with Identiy as the initial
         * transform
         *
         * @param name [in] name of the frame
         */
        explicit MovableFrame(const std::string& name);

        /**
         * @brief Sets the transform in the state
         * @param transform [in] transform to set
         * @param state [out] state into which to set the transform
         */
        void setTransform(const math::Transform3D<>& transform, State& state);

    private:
        void doMultiplyTransform(const math::Transform3D<>& parent,
                            const State& state,
                            math::Transform3D<>& result) const;

        math::Transform3D<> doGetTransform(const State& state) const;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
