#ifndef RW_MODELS_COLLISIONMODELINFO_HPP_
#define RW_MODELS_COLLISIONMODELINFO_HPP_

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

#endif /*CollisionModelInfo_HPP_*/
