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
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<FixedFrame> Ptr;

        /**
         * @brief A frame fixed to its parent with a constant relative transform
         * of \b transform.
         *
         * @param name [in] The name of the frame.
         * @param transform [in] The transform with which to attach the frame.
         */
        FixedFrame(const std::string& name,
			const rw::math::Transform3D<>& transform);

        //! @brief destructor
        virtual ~FixedFrame(){}

        /**
         * @brief Sets the fixed transform of this frame.
         * @param transform [in] the new transformation of this frame
         * @note THIS IS NOT THREAD SAFE. If you need thread safety then use
         * MovableFrame instead or make sure multiple threads are not using this
         * frame when changing the transformation.
         */
		void setTransform(const rw::math::Transform3D<>& transform);

		/**
		 * @brief Move the frame such that it is located with a relative transform \b refTtarget relative to \b refframe.
		 * @param refTtarget [in] the transform relative to \b refframe .
		 * @param refframe [in] the reference frame.
		 * @param state [in] the state giving the current poses.
		 */
		void moveTo(const rw::math::Transform3D<>& refTtarget, Frame* refframe, State& state);

        /**
         * @brief get the fixed transform of this frame.
         */
        const math::Transform3D<>& getFixedTransform() const;

    private:
        void doMultiplyTransform(const math::Transform3D<>& parent,
                                 const State& state,
								 rw::math::Transform3D<>& result) const;

        rw::math::Transform3D<> doGetTransform(const State& state) const;

    private:
        rw::math::Transform3D<> _transform;
    };

    /*@}*/

}} // end namespaces

#endif // end include guard
