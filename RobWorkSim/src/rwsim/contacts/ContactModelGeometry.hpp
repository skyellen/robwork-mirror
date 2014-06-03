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

#ifndef RWSIM_CONTACTS_CONTACTMODELGEOMETRY_HPP_
#define RWSIM_CONTACTS_CONTACTMODELGEOMETRY_HPP_

/**
 * @file ContactModelGeometry.hpp
 *
 * \copydoc rwsim::contacts::ContactModelGeometry
 */

#include "ContactModel.hpp"
#include "ContactStrategy.hpp"

namespace rwsim {
namespace contacts {
//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief Generic contact model for strategies that find contacts between two different types of geometry.
 */
template <class A, class B>
class ContactModelGeometry: public rwsim::contacts::ContactModel {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<ContactModelGeometry<A, B> > Ptr;

	//! @brief Model for each geometry.
	template <class T>
	struct GeometryModel {
		//! The geometry id.
		std::string geoId;

		//! Pointer to the geometry.
		T geo;

		//! The location of the geometry.
		rw::math::Transform3D<> transform;

		//! The frame
		const rw::kinematics::Frame* frame;
	};

	//! @brief The type of GeometryModel for geometry A.
	typedef GeometryModel<A> TypeA;

	//! @brief The type of GeometryModel for geometry B.
	typedef GeometryModel<B> TypeB;

	/**
	 * @brief Construct new model for contacts between geometries.
	 * @param owner [in] the strategy that owns this model.
	 */
	ContactModelGeometry(ContactStrategy *owner): ContactModel(owner) {}

	//! @brief Destructor
	virtual ~ContactModelGeometry() {};

	//! @copydoc rwsim::contacts::ContactModel::getName
	virtual std::string getName() const { return "ContactModelGeometry"; };

public:
	//! @brief List of geometry models belonging to this model.
	std::vector<GeometryModel<A> > modelsA;

	//! @brief List of hole models belonging to this model.
	std::vector<GeometryModel<B> > modelsB;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_CONTACTMODELGEOMETRY_HPP_ */
