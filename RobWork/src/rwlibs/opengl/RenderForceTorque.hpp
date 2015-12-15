/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWLIBS_OPENGL_RENDERFORCETORQUE_HPP
#define RWLIBS_OPENGL_RENDERFORCETORQUE_HPP

/**
 * @file RenderForceTorque.hpp
 *
 * \copydoc rwlibs::opengl::RenderForceTorque
 */

#include <rwlibs/os/rwgl.hpp>

#include <rw/graphics/Render.hpp>
#include <rw/math/EAA.hpp>

namespace rwlibs {
namespace opengl {
//! @addtogroup opengl

//! @{
/**
 * @brief RenderForceTorque makes a visualization of forces and torques.
 */
class RenderForceTorque : public rw::graphics::Render
{
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<RenderForceTorque> Ptr;

	//! @brief Constructs a RenderForceTorque
	RenderForceTorque();

	//! @brief Destructor
	virtual ~RenderForceTorque();

	/**
	 * @brief Get the force currently set.
	 * @return the force.
	 */
	rw::math::Vector3D<> getForce() const;

	/**
	 * @brief Get the torque currently set.
	 * @return the torque.
	 */
	rw::math::Vector3D<> getTorque() const;

	/**
	 * @brief Get the current scaling of the force vector in meters per Newton.
	 * @return the scale factor
	 */
	float getScaleForce() const;

	/**
	 * @brief Get the current scaling of the torque vector in meters per Newton meter.
	 * @return the scale factor
	 */
	float getScaleTorque() const;

	/**
	 * @brief Set the force.
	 * @param force [in] the force
	 */
	void setForce(rw::math::Vector3D<> force);

	/**
	 * @brief Set the torque.
	 * @param torque [in] the torque
	 */
	void setTorque(rw::math::Vector3D<> torque);

	/**
	 * @brief Set the current scaling of the force vector in meters per Newton.
	 * @param scale [in] the scale factor
	 */
	void setScaleForce(float scale);

	/**
	 * @brief Set the current scaling of the torque vector in meters per Newton meter.
	 * @param scale [in] the scale factor
	 */
	void setScaleTorque(float scale);

	/**
	 * @brief Set the current scaling of the force and torque arrows in meters per unit.
	 * @param scaleForce [in] the scale factor for force
	 * @param scaleTorque [in] the scale factor for torque
	 */
	void setScales(float scaleForce, float scaleTorque);

	/**
	 * @brief Get the current color used for the force arrow.
	 * @return the color as (r,g,b) vector.
	 */
	rw::math::Vector3D<float> getColorForce() const;

	/**
	 * @brief Get the current color used for the torque arrow.
	 * @return the color as (r,g,b) vector.
	 */
	rw::math::Vector3D<float> getColorTorque() const;

    /**
     * @brief Sets color of the force arrow
     * @param r [in] red color component
     * @param g [in] green color component
     * @param b [in] blue color component
     */
    void setColorForce(float r, float g, float b);

    /**
     * @brief Sets color of the torque arrow
     * @param r [in] red color component
     * @param g [in] green color component
     * @param b [in] blue color component
     */
    void setColorTorque(float r, float g, float b);

	/* Functions inherited from Render */

	//! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
	void draw(const rw::graphics::DrawableNode::RenderInfo& info,
			rw::graphics::DrawableNode::DrawType type,
			double alpha) const;

private:
	rw::math::Vector3D<> _force, _torque;
	float _scaleForce, _scaleTorque;
	rw::math::Vector3D<float> _colorForce, _colorTorque;
	mutable GLUquadricObj *_quadratic;
};
//! @}
} /* namespace opengl */
} /* namespace rwlibs */
#endif /* RWLIBS_OPENGL_RENDERFORCETORQUE_HPP */
