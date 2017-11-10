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

#ifndef RW_MODELS_UNIVERSALJOINT_HPP_
#define RW_MODELS_UNIVERSALJOINT_HPP_

/**
 * @file UniversalJoint.hpp
 *
 * \copydoc rw::models::UniversalJoint
 */

#include "Joint.hpp"

namespace rw {
namespace models {
//! @addtogroup models

//! @{
/**
 * @brief A universal joint that allows rotations in two directions.
 *
 * Rotation is allowed around the x and y axes. The position and rotation around the z axis is fixed.
 */
class UniversalJoint: public Joint {
public:
	//! @brief Smart pointer type of UniversalJoint
	typedef rw::common::Ptr<UniversalJoint> Ptr;

    /**
     * @brief Construct a universal joint.
     * @param name [in] name of the joint.
     * @param transform [in] static transform of the joint.
     */
	UniversalJoint(const std::string& name, const rw::math::Transform3D<>& transform);

	//! @brief Destructor.
	virtual ~UniversalJoint();

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

#endif /* RW_MODELS_UNIVERSALJOINT_HPP_ */
