#ifndef RW_MODELS_DRAWABLEMODELINFO_HPP_
#define RW_MODELS_DRAWABLEMODELINFO_HPP_

#include <iostream>
#include <rw/math/Transform3D.hpp>

namespace rw {
namespace models {

/**
 * @brief this is a light weight container class for varius information regarding 
 * drawable models. The drawable information in this class is regarded const.
 */
class DrawableModelInfo {
private:
	
    /**
     * @brief constructor
     */
	DrawableModelInfo() {}
	
public:	
	/**
	 * @brief constructor
	 * @param id [in] string idetifier of this drawable
	 */
	DrawableModelInfo(const std::string& id);

   /**
     * @brief constructor
     * @param id [in] string idetifier of this drawable
     */
	DrawableModelInfo(const std::string& id, rw::math::Transform3D<> t3d);

   /**
     * @brief constructor
     * @param id [in] string idetifier of this drawable
     */
	DrawableModelInfo(const std::string& id, rw::math::Transform3D<> t3d,
				 double scale, bool wire, bool high);

	/**
	 * @brief destructor
	 */
	virtual ~DrawableModelInfo() {};
	
	/**
	 * @brief gets if this drawable is initially highlighted
	 */
	bool isHighlighted() const {
		return _highlighted;
	}
	
	/**
	 * @brief gets if this drawable initially is to be drawn in Wire mode
	 */
	bool isWireMode() const {
		return _wireMode;
	}
	
	/**
	 * @brief gets the string identifier of this drawable info
	 */
	const std::string& getId() const {
		return _drawableId;
	}
	
	/**
	 * @brief gets the geometric scale of this drawable
	 */
	double getGeoScale() const {
		return _geoScale;
	}
	
	/**
	 * @brief gets the transform of this drawable
	 */
	const rw::math::Transform3D<>& getTransform() const {
		return _transform;
	}
private:
    
    std::string _drawableId;
    rw::math::Transform3D<> _transform;
    double _geoScale;
    bool _wireMode;
    bool _highlighted;
};

}
}

#endif /*DrawableModelInfo_HPP_*/
