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
	if(initQ.size() != 8)
		RW_THROW("Size of parameter list must equal 8!");
		
	_length = initQ(0);
	_width = initQ(1);
	_depth = initQ(2);
	_chamferDepth = initQ(3);
	_chamferAngle = initQ(4);
	_cutPosition = initQ(5);
	_cutDepth = initQ(6);
	_cutAngle = initQ(7);
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
	CSGModel cutout = CSGModel::makeWedge(_cutAngle);
	base -= cutout.rotate(-90*Deg2Rad, 90*Deg2Rad, 0).translate(_cutPosition, 0, _cutDepth);
	
	TriMesh::Ptr mesh = base.getTriMesh();

	return mesh;
}

rw::math::Q JawPrimitive::getParameters() const
{
	Q q(8);
	
	q(0) = _length;
	q(1) = _width;
	q(2) = _depth;
	q(3) = _chamferDepth;
	q(4) = _chamferAngle;
	q(5) = _cutPosition;
	q(6) = _cutDepth;
	q(7) = _cutAngle;
	
	return q;
}



std::string JawPrimitive::toString() const
{
	stringstream str;
	
	str << _length << " " << _width << " " << _depth << " " << _chamferDepth << " " << Rad2Deg*_chamferAngle;
	str << " " << _cutPosition << " " << _cutDepth << " " << Rad2Deg*_cutAngle;
	
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

