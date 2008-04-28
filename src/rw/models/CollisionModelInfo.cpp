#include "CollisionModelInfo.hpp"

using namespace rw::models;

CollisionModelInfo::CollisionModelInfo(const std::string& id, double scale):
	_colId(id), 
	_transform(rw::math::Transform3D<>::identity()),
	_geoScale(scale)
{
}	
	
CollisionModelInfo::CollisionModelInfo(const std::string& id, 
			rw::math::Transform3D<> t3d, double scale):
	_colId(id), 
	_transform(t3d),
	_geoScale(scale)
{
}
