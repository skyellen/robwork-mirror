/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_MODELS_SPHERICALJOINT_HPP_
#define RW_MODELS_SPHERICALJOINT_HPP_

/**
 * @file SphericalJoint.hpp
 *
 * \copydoc rw::models::SphericalJoint
 */

#include "Joint.hpp"

namespace rw {
namespace models {
//! @addtogroup models

//! @{
/**
 * @brief A spherical joint that allows rotations in all directions.
 *
 * Rotation is allowed around the x-, y- and z-axes. The position is fixed.
 */
class SphericalJoint: public Joint {
public:
	//! @brief Smart pointer type of SphericalJoint
	typedef rw::common::Ptr<SphericalJoint> Ptr;

    /**
     * @brief Construct a spherical joint.
     * @param name [in] name of the joint.
     * @param transform [in] static transform of the joint.
     */
	SphericalJoint(const std::string& name, const rw::math::Transform3D<>& transform);

	//! @brief Destructor.
	virtual ~SphericalJoint();

	// From Frame
	//! @brief Frame::doMultiplyTransform
    virtual void doMultiplyTransform(const rw::math::Transform3D<>& parent,
                                     const rw::kinematics::State& state,
									 rw::math::Transform3D<>& result) const;

	//! @brief Frame::doGetTransform
    virtual math::Transform3D<> doGetTransform(const rw::kinematics::State& state) const;

    // From Joint
	//! @copydoc Joint::getJacobian
	virtual void getJacobian(std::size_t row,
			std::size_t col,
			const rw::math::Transform3D<>& joint,
			const rw::math::Transform3D<>& tcp,
			const rw::kinematics::State& state,
			rw::math::Jacobian& jacobian) const;

	//! @copydoc Joint::getFixedTransform
	virtual rw::math::Transform3D<> getFixedTransform() const;

	//! @copydoc Joint::setFixedTransform
	virtual void setFixedTransform(const rw::math::Transform3D<>& t3d);

	//! @copydoc Joint::getJointTransform
	virtual rw::math::Transform3D<> getJointTransform(const rw::kinematics::State& state) const;

	//! @copydoc Joint::setJointMapping
	virtual void setJointMapping(rw::math::Function1Diff<>::Ptr function);

	//! @copydoc Joint::removeJointMapping
	virtual void removeJointMapping();

private:
	rw::math::Transform3D<> _T;
	rw::math::Function1Diff<>::Ptr _mapping;
};
//! @}
} /* namespace models */
} /* namespace rw */

#endif /* RW_MODELS_SPHERICALJOINT_HPP_ */
