#include "DrawableModelInfo.hpp"

using namespace rw::models;

namespace {
	
	void init(double &scale, bool &wire, bool &high){
		scale = 1.0;
		wire = false;
		high = false;
	}

}

DrawableModelInfo::DrawableModelInfo(const std::string& id):
	_drawableId(id), 
	_transform(rw::math::Transform3D<>::Identity())
{		
	init(_geoScale,_wireMode,_highlighted);
}
	
DrawableModelInfo::DrawableModelInfo(const std::string& id, rw::math::Transform3D<> t3d):
	_drawableId(id), 
	_transform(t3d)
{
	init(_geoScale,_wireMode,_highlighted);
}

DrawableModelInfo::DrawableModelInfo(const std::string& id, rw::math::Transform3D<> t3d,
			 double scale, bool wire, bool high):
	_drawableId(id),
	_transform(t3d),
	_geoScale(scale),
	_wireMode(wire),
	_highlighted(high)
{
	
}
