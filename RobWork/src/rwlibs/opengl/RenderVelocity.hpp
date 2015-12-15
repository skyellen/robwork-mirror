/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWLIBS_OPENGL_RENDERVELOCITY_HPP_
#define RWLIBS_OPENGL_RENDERVELOCITY_HPP_

/**
 * @file RenderVelocity.hpp
 *
 * \copydoc rwlibs::opengl::RenderVelocity
 */

#include <rw/graphics/Render.hpp>
#include <rw/math/VelocityScrew6D.hpp>
#include <rwlibs/os/rwgl.hpp>

namespace rwlibs {
namespace opengl {
//! @addtogroup opengl

//! @{
//! @brief Render for linear and angular velocity.
class RenderVelocity: public rw::graphics::Render {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<RenderVelocity> Ptr;

	//! @brief Constructor
	RenderVelocity();

	//! @brief Destructor
	virtual ~RenderVelocity();

	//! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
	void draw(const rw::graphics::DrawableNode::RenderInfo& info,
			rw::graphics::DrawableNode::DrawType type, double alpha) const;

	/**
	 * @brief Get the currently set velocity.
	 * @return the velocity.
	 */
	rw::math::VelocityScrew6D<> getVelocity() const;

	/**
	 * @brief Get the current scaling of the linear velocity in meters per meter per second.
	 * @return the scale factor.
	 */
	float getScaleLinear() const;

	/**
	 * @brief Get the angular scaling of the angular velocity in meters per radians per second.
	 * @return the scale factor.
	 */
	float getScaleAngular() const;

	/**
	 * @brief Set the velocity.
	 * @param velocity [in] the velocity.
	 */
	void setVelocity(rw::math::VelocityScrew6D<> velocity);

	/**
	 * @brief Set the scaling of the linear velocity in meters per meter per second.
	 * @param scale [in] the scale factor.
	 */
	void setScaleLinear(float scale);

	/**
	 * @brief Set the angular scaling of the angular velocity in meters per radians per second.
	 * @param scale [in] the scale factor.
	 */
	void setScaleAngular(float scale);

	/**
	 * @brief Set the current scaling of the linar and angular velocity arrows.
	 * @param scaleLinear [in] the scale factor for linear velocity.
	 * @param scaleAngular [in] the scale factor for angular velocity.
	 */
	void setScales(float scaleLinear, float scaleAngular);

	/**
	 * @brief Get the current color used for the linear velocity arrow.
	 * @return the color as (r,g,b) vector.
	 */
	rw::math::Vector3D<float> getColorLinear() const;

	/**
	 * @brief Get the current color used for the angular velocity arrow.
	 * @return the color as (r,g,b) vector.
	 */
	rw::math::Vector3D<float> getColorAngular() const;

	/**
	 * @brief Sets color of the linear velocity arrow.
	 * @param r [in] red color component.
	 * @param g [in] green color component.
	 * @param b [in] blue color component.
	 */
	void setColorLinear(float r, float g, float b);

	/**
	 * @brief Sets color of the angular velocity arrow.
	 * @param r [in] red color component.
	 * @param g [in] green color component.
	 * @param b [in] blue color component.
	 */
	void setColorAngular(float r, float g, float b);

	/**
	 * @brief Set the resolution of the rendering.
	 *
	 * The round parts will be discretized in the given resolution.
	 * Default value is 32.
	 *
	 * @param resolution
	 */
	void setResolution(unsigned int resolution);

private:
	rw::math::VelocityScrew6D<> _velocity;
	float _scaleLin, _scaleAng;
	rw::math::Vector3D<float> _colorLinear, _colorAngular;
	unsigned int _resolution;
	mutable GLUquadricObj *_quadratic;
};
//! @}
} /* namespace opengl */
} /* namespace rwlibs */
#endif /* RWLIBS_OPENGL_RENDERVELOCITY_HPP_ */
