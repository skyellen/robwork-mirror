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

#ifndef RWSIM_DRAWABLE_RENDERCIRCLES_HPP_
#define RWSIM_DRAWABLE_RENDERCIRCLES_HPP_

//! @file RenderCircles.hpp

#include <vector>

#include <rw/graphics/Render.hpp>

namespace rwsim { namespace util { class CircleModel; } }

namespace rwsim {
namespace drawable {
	//! @addtogroup rwsim_drawable
	//! @{
	/**
	 * @brief Render a set of circles
	 */
	class RenderCircles: public rw::graphics::Render
	{
	public:
		/**
		 * @brief constructor
		 * @param angleResolution [in] the resolution of the circle line segments
		 * in degree. The circle is approximated using line segments.
		 */
		RenderCircles(float angleResolution=10.0);

		/**
		 * @brief destructor
		 */
		virtual ~RenderCircles();

		/**
		 * @brief adds circle to the circles that are allready drawn
		 * @param circle [in] circle to draw
		 */
		void addCircle(const util::CircleModel& circle);

		/**
		 * @brief adds circles to the circles that are allready drawn
		 */
		void addCircles(const std::vector<util::CircleModel>& circles);

		/**
		 * @brief set the circles that is to be rendered
		 * @param circles [in] the vector of circles
		 */
		void setCircles(const std::vector<util::CircleModel>& circles);

		/**
		 * @brief set the color used for the model
		 * @param r [in] red color value
		 * @param g [in] green color value
		 * @param b [in] blue color value
		 */
		void setColor(double r, double g, double b);

		/**
		 * @brief clear the list of circles
		 */
		void clear();

        //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
        void draw(const rw::graphics::DrawableNode::RenderInfo& info,
                  rw::graphics::DrawableNode::DrawType type,
                  double alpha) const;

	private:
		float _stepSize;
		std::vector<util::CircleModel> _circles;
		float _color[3];
	};

	//! @}
}
}

#endif /*RenderGhost_HPP_*/
