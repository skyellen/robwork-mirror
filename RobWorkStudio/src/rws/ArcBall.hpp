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


#ifndef RWS_ARCBALL_HPP
#define RWS_ARCBALL_HPP

#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>
#include "CameraController.hpp"

namespace rws{

	/**
	 * @brief Use the ArcBall method to control the camera view point in a scene.
	 *
	 * The ArcBall method
	 * defines a way to map a 2d position into a 3d position on some sphere.
	 * This means that the rotation is defined by dragging a point on a sphere
	 * around its center.
	 */
	class ArcBall: public CameraController {
	protected:
		/**
		 * @brief maps a 2d position (x,y) into a position on a 3D sphere
		 * which is centered around \b _centerPt.
		 * @param x [in]
		 * @param y [in]
		 */
		rw::math::Vector3D<float> mapToSphere(float x, float y) const;

	public:
		/**
		 * @brief constructor
		 */
		ArcBall(GLfloat NewWidth, GLfloat NewHeight);

		/**
		 * @brief destructor
		 */
		virtual ~ArcBall() { /* nothing to do */ };

		/**
		 * @brief set the bounds that define the area where the 2d point
		 * is valid. The bound is defined in a plane with [0,width] and
		 * [0,height]. Where (0,0) is the upper left corner of the plane.
		 * @param NewWidth [in] width
		 * @param NewHeight [in] height
		 */
		inline void setBounds(GLfloat NewWidth, GLfloat NewHeight)
		{
			// Set adjustment factor for width/height
			_adjustWidth  = 1.0f / ((NewWidth  - 1.0f) * 0.5f);
			_adjustHeight = 1.0f / ((NewHeight - 1.0f) * 0.5f);
		}

		/**
		 * @copydoc CameraController::setCenter
		 */
		void setCenter(float x, float y){
			_centerPt[0] = x;
			_centerPt[1] = y;
		}

		/**
		 * @copydoc CameraController::click
		 */
		void click(float x, float y);

		/**
		 * @brief draw the ArcBall
		 */
		void draw();

		/**
		 * @copydoc CameraController::drag
		 */
		rw::math::Quaternion<float> drag(float x, float y);

	private:
		rw::math::Vector2D<float> _centerPt; // Center of the ball
		rw::math::Vector3D<float> _stVec;          // Saved click vector
		rw::math::Vector3D<float> _enVec;          // Saved drag vector
		GLfloat _adjustWidth;    // Mouse bounds width
		GLfloat _adjustHeight;   // Mouse bounds height
		float _arcballRadi; // radius of arcball
	};

}

#endif
