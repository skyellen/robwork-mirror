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

#ifndef RWSIM_CONTACTS_CONTACTSTRATEGYPQP_HPP_
#define RWSIM_CONTACTS_CONTACTSTRATEGYPQP_HPP_

#include "ContactStrategy.hpp"
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>

/**
 * @file ContactStrategyPQP.hpp
 *
 * \copydoc rwsim::contacts::ContactStrategyPQP
 */

namespace rwsim {
namespace contacts {
//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief Detection of contacts between triangle meshes.
 */
class ContactStrategyPQP: public rwsim::contacts::ContactStrategy {
public:
	/**
	 * @brief Strategy used for condensing contacts.
	 */
	enum ContactFilter {
		NONE,//!< No filtering.
		MANIFOLD//!< Filtering with manifold.
	};

	/**
	 * @brief Create new strategy.
	 */
	ContactStrategyPQP();

	/**
	 * @brief Destructor
	 */
	virtual ~ContactStrategyPQP();

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::match
	 */
	virtual bool match(rw::geometry::GeometryData::Ptr geoA, rw::geometry::GeometryData::Ptr geoB);

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::findContacts(rw::proximity::ProximityModel*,const rw::math::Transform3D<>&,rw::proximity::ProximityModel*,const rw::math::Transform3D<>&)
	 */
	virtual std::vector<Contact> findContacts(
			rw::proximity::ProximityModel* a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel* b,
			const rw::math::Transform3D<>& wTb);

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::findContacts(rw::proximity::ProximityModel*,const rw::math::Transform3D<>&,rw::proximity::ProximityModel*,const rw::math::Transform3D<>&,ContactStrategyData&)
	 */
	virtual std::vector<Contact> findContacts(
			rw::proximity::ProximityModel* a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel* b,
			const rw::math::Transform3D<>& wTb,
			ContactStrategyData &data);

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::getName
	 */
	virtual std::string getName();

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::createModel
	 */
	virtual rw::proximity::ProximityModel::Ptr createModel();

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::destroyModel
	 */
	virtual void destroyModel(rw::proximity::ProximityModel* model);

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::addGeometry(rw::proximity::ProximityModel*,const rw::geometry::Geometry&)
	 */
	virtual bool addGeometry(rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom);

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::addGeometry(rw::proximity::ProximityModel*,rw::geometry::Geometry::Ptr,bool)
	 */
	virtual bool addGeometry(rw::proximity::ProximityModel* model, rw::geometry::Geometry::Ptr geom, bool forceCopy=false);

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::removeGeometry
	 */
	virtual bool removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId);

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::getGeometryIDs
	 */
	virtual std::vector<std::string> getGeometryIDs(rw::proximity::ProximityModel* model);

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::clear
	 */
	virtual void clear();

	/**
	 * @brief If strategy should convert all geometry to meshes, or only work on geometry that is already meshes.
	 *
	 * @param matchAll [in] true if strategy should match all geometries (default), false otherwise.
	 */
	virtual void setMatchAll(bool matchAll = true);

	/**
	 * @brief Set the method used to filter contacts (condenses many contacts into fewer contacts).
	 *
	 * @param filter [in] the strategy to use - default is the OBRManifold-strategy.
	 */
	virtual void setContactFilter(ContactFilter filter = MANIFOLD);

	/**
	 * @brief The current distance threshold used by the simulator.
	 *
	 * The default threshold is 0.5 mm.
	 * This can be changed by setting the ContactStrategyPQPThreshold property in the PropertyMap.
	 * If this is not set, the MaxSepDistance property can also be used.
	 *
	 * @return the threshold (positive).
	 */
	virtual double getThreshold() const;

    /**
     * @brief For modelling of a trimesh.
     */
	struct Model {
    	//! The geometry id of the mesh.
		std::string geoId;

		//! Pointer to the trimesh.
		rw::geometry::TriMesh::Ptr mesh;

		//! The location of the mesh.
		rw::math::Transform3D<> transform;

		//! The frame
		const rw::kinematics::Frame* frame;
	};

	/**
	 * @brief The contact model used by this strategy.
	 */
	class TriMeshModel: public ContactModel {
	public:
		/**
		 * @brief Construct new model for object consisting of a trimesh.
		 *
		 * @param owner [in] the strategy that owns this model.
		 */
		TriMeshModel(ContactStrategy *owner):
			ContactModel(owner)
		{
		}

		/**
		 * @copydoc rwsim::contacts::ContactModel::getName
		 */
		virtual std::string getName() const { return "TriMeshModel"; };

		/**
		 * @brief List of trimesh models belonging to this model.
		 */
		std::vector<Model> models;

		/**
		 * @brief Proximity Model for use with collision detector.
		 */
		ProximityModel::Ptr pmodel;
	};

private:
	std::vector<Contact> manifoldFilter(const std::vector<Contact> &contacts);

	bool _matchAll;
	rwlibs::proximitystrategies::ProximityStrategyPQP *_narrowStrategy;
	ContactFilter _filtering;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_CONTACTSTRATEGYPQP_HPP_ */
