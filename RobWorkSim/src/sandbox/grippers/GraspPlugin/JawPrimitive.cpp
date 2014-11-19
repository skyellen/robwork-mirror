#include "JawPrimitive.hpp"

#include <sstream>
#include <rw/common/Ptr.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include "CSGModel.hpp"



using namespace std;
using namespace rw::geometry;
using namespace rw::math;
using namespace rw::common;
using namespace rw::csg;



JawPrimitive::JawPrimitive(const rw::math::Q& initQ)
{
	if(!(initQ.size() == 10 || initQ.size() == 11)) {
		RW_THROW("Size of parameter list must equal 10 or 11 (cut tilt)!");
	}
	
	int i = 0;
	_type = CutoutType(initQ(i++));
	_length = initQ(i++);
	_width = initQ(i++);
	_depth = initQ(i++);
	_chamferDepth = initQ(i++);
	_chamferAngle = initQ(i++);
	_cutPosition = initQ(i++);
	_cutDepth = initQ(i++);
	_cutAngle = initQ(i++);
	_cutRadius = initQ(i++);
	
	if (initQ.size() == 11) {
		_cutTilt = initQ(i++);
	}
}



JawPrimitive::~JawPrimitive()
{
}



TriMesh::Ptr JawPrimitive::createMesh(int resolution) const
{
	//use CSG
	CSGModel base = CSGModel::makeBox(_length, _depth, _width).translate(_length/2, 0, _width/2);
	
	// make chamfer
	Vector3D<> point(_length, 0.0, (1-_chamferDepth)*_width);
	Transform3D<> t(Vector3D<>(), RPY<>(0, -_chamferAngle, 0).toRotation3D());
	Vector3D<> normal = t * Vector3D<>::x();
	base -= CSGModel::makePlane(point, -normal);
	
	// make cutout
	if (_type == Cylindrical) {
		CSGModel cutout = CSGModel::makeCylinder(_cutRadius, 100.0);
		base -= cutout.rotate(-90*Deg2Rad, 90*Deg2Rad, 0).rotate(_cutTilt, 0, 0).translate(_cutPosition, 0, _cutDepth-_cutRadius);
	} else {
		CSGModel cutout = CSGModel::makeWedge(_cutAngle);
		base -= cutout.rotate(-90*Deg2Rad, 90*Deg2Rad, 0).rotate(_cutTilt, 0, 0).translate(_cutPosition, 0, _cutDepth);
	}
	
	TriMesh::Ptr mesh = base.getTriMesh();

	return mesh;
}

rw::math::Q JawPrimitive::getParameters() const
{
	Q q(11);
	
	int i = 0;
	q(i++) = _type;
	q(i++) = _length;
	q(i++) = _width;
	q(i++) = _depth;
	q(i++) = _chamferDepth;
	q(i++) = _chamferAngle;
	q(i++) = _cutPosition;
	q(i++) = _cutDepth;
	q(i++) = _cutAngle;
	q(i++) = _cutRadius;
	q(i++) = _cutTilt;
	
	return q;
}



std::string JawPrimitive::toString() const
{
	stringstream str;
	
	str << _type << " " << _length << " " << _width << " " << _depth << " " << _chamferDepth << " " << Rad2Deg*_chamferAngle;
	str << " " << _cutPosition << " " << _cutDepth << " ";
	if (_type == Cylindrical) {
		str << _cutRadius;
	} else {
		str << Rad2Deg*_cutAngle;
	}
	str << " " << Rad2Deg*_cutTilt;
	
	return str.str();
}



/*std::ostream& operator<<(std::ostream& stream, const rw::geometry::JawPrimitive& jaw)
{
	stream << "Jaw {\n";
	stream << "- box (" << jaw._length << ", " << jaw._width << ", " << jaw._depth << ")\n";
	stream << "- chamfer (" << jaw._chamferDepth << ", " << Rad2Deg * jaw._chamferAngle << "deg)\n";
	stream << "- cut (" << jaw._cutPosition << ", " << jaw._cutDepth << ", " << Rad2Deg * jaw._cutAngle << "deg)\n";
	stream << "}" << endl;
	
	return stream;
}*/

