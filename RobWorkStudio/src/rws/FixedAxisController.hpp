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


#ifndef RWS_FIXEDAXISCONTROLLER_HPP
#define RWS_FIXEDAXISCONTROLLER_HPP

#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>
#include "CameraController.hpp"

namespace rws{

	/**
	 * @brief the FixedAxis Camera Controller is used to control the camera position
	 * and orientation in relation to a pivot point where the rotations generated are
	 * around the fixed axis of the world.
	 *
	 * More specifically the horizontal mouse drag rotates the scene around the world z-axis
	 * whereas the vertical drag movements rotate the scene around the camera y-axis located
	 * in the pivot point.
	 */
	class FixedAxisController: public CameraController {
	public:
		/**
		 * @brief constructor
		 */
	    FixedAxisController(GLfloat NewWidth, GLfloat NewHeight);

		/**
		 * @brief destructor
		 */
		virtual ~FixedAxisController() { /* nothing to do */ };

		//! @copydoc CameraController::setBounds
		void setBounds(GLfloat NewWidth, GLfloat NewHeight);

		//! @copydoc CameraController::setCenter
        void setCenter(const rw::math::Vector3D<>& center, const rw::math::Vector2D<>& screenCenter);

        //! @copydoc CameraController::getCenter
        rw::math::Vector3D<> getCenter(){return _pivotPoint;}

        //! @copydoc CameraController::draw
        void draw();

        //! @copydoc CameraController::handleEvent
        virtual void handleEvent(QEvent* event);

        //! @copydoc CameraController::getTransform
        rw::math::Transform3D<> getTransform() const;

        //! @copydoc CameraController::setTransform
        void setTransform(const rw::math::Transform3D<>& t3d);

	private:
		rw::math::Vector2D<float> _centerPt; // Center of the ball
		rw::math::Vector3D<float> _stVec;          // Saved click vector
		rw::math::Vector3D<float> _enVec;          // Saved drag vector
		GLfloat _adjustWidth;    // Mouse bounds width
		GLfloat _adjustHeight;   // Mouse bounds height
		float _arcballRadi; // radius of arcball
		double _height, _width;
		rw::math::Transform3D<> _viewTransform;
		//rw::math::Rotation3D<> _viewRotation;
		rw::math::Vector3D<> _lastPos,_pivotPoint;
		double _zoomFactor, _zoomScale;
		rw::math::Transform3D<> _camTransform;

	};

}

#endif
