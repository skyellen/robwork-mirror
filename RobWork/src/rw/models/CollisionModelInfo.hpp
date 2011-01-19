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
	CollisionModelInfo(const std::string& id, const std::string& name, double scale = 1.0);

    /**
     * @brief constructor
     * @param id [in] the string id of this collisionmodel, typicly a filename or primitive prefixed with #
     * @param t3d [in] the static transformation of the collisionmodel
     * @param scale [in] the geometric scale of the collisionmodel
     */
	CollisionModelInfo(const std::string& id, const std::string& name, rw::math::Transform3D<> t3d, double scale = 1.0);

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

    const std::string& getName() const {
        return _name;
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
    std::string _colId, _name;
    rw::math::Transform3D<> _transform;
    double _geoScale;
};

}
}

#endif /*RW_MODELS_COLLISIONMODELINFO_HPP*/
