/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RW_MODELS_COLLISIONMODELINFO_HPP
#define RW_MODELS_COLLISIONMODELINFO_HPP

#include <iostream>
#include <rw/math/Transform3D.hpp>

namespace rw {
namespace models {

/**
 * @brief this is a light weight container class for varius information regarding
 * collision models. The collision information in this class is regarded const.
 */
class CollisionModelInfo {
private:
	/**
	 * @brief constructor
	 */
	CollisionModelInfo(){};

public:

    /**
     * @brief constructor
     * @param id [in] the string id of this collisionmodel, typicly a filename or primitive prefixed with #
     * @param scale [in] the geometric scale of the collisionmodel
     */
	CollisionModelInfo(const std::string& id, double scale = 1.0);

    /**
     * @brief constructor
     * @param id [in] the string id of this collisionmodel, typicly a filename or primitive prefixed with #
     * @param t3d [in] the static transformation of the collisionmodel
     * @param scale [in] the geometric scale of the collisionmodel
     */
	CollisionModelInfo(const std::string& id, rw::math::Transform3D<> t3d, double scale = 1.0);

	/**
	 * @brief destructor
	 */
	virtual ~CollisionModelInfo() {};

	/**
	 * @brief gets the string identifier for the collision model
	 * @return string identifying the collisionmodel
	 */
	const std::string& getId() const {
		return _colId;
	}

	/**
	 * @brief gets the geometric scale of the collision model.
	 */
	double getGeoScale() const {
		return _geoScale;
	}

	/**
	 * @brief gets the transformation of this collisionmodel
	 */
	const rw::math::Transform3D<>& getTransform() const {
		return _transform;
	}

private:
    std::string _colId;
    rw::math::Transform3D<> _transform;
    double _geoScale;
};

}
}

#endif /*RW_MODELS_COLLISIONMODELINFO_HPP*/
