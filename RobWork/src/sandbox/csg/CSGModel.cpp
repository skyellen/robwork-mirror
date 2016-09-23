#include "CSGModel.hpp"

#include <rw/rw.hpp>
#include <rw/loaders/model3d/STLFile.hpp>
#include "CSGConvert.hpp"



using namespace std;
using namespace rw;
using namespace rw::common;
using namespace rw::math;
using namespace rw::geometry;
using namespace rw::csg;
using namespace rw::loaders;



//----------------------------------------------------------------------
CSGModel::CSGModel() :
	_needsConversion(false)
{}



//----------------------------------------------------------------------
CSGModel::CSGModel(const CSGModel& csgmodel) :
	_needsConversion(csgmodel._needsConversion),
	_model(csgmodel._model)
{
	if (csgmodel._mesh != NULL) {
		_mesh = csgmodel._mesh->clone();
	}
}



//----------------------------------------------------------------------
CSGModel::CSGModel(const TriMesh& trimesh) :
	_needsConversion(false),
	_mesh(trimesh.clone())
{
	_model = *CSGConvert::TriMesh2csgjs_model(trimesh);
}



//----------------------------------------------------------------------
CSGModel& CSGModel::makeCube(float x, float y, float z)
{
	Box box(x, y, z);
	
	CSGModel* csgmodel = new CSGModel(*(box.createMesh(0)));
	
	return *csgmodel;
}



//----------------------------------------------------------------------
CSGModel& CSGModel::makeCylinder(float r, float h)
{
	Cylinder cyl(r, h);
	
	CSGModel* csgmodel = new CSGModel(*(cyl.createMesh(32)));
	
	return *csgmodel;
}



//----------------------------------------------------------------------
CSGModel& CSGModel:: makeSphere(float r)
{
	Sphere sph(r, 4);
	
	CSGModel* csgmodel = new CSGModel(*(sph.createMesh(3)));
	
	return *csgmodel;
}



//----------------------------------------------------------------------
CSGModel& CSGModel::makePlane(Vector3D<> point, Vector3D<> normal)
{
	const float planeSize = 100.0;
	
	normal = normalize(normal);
	Transform3D<> t = Transform3D<>::makeLookAt(point, point+normal, Vector3D<>::y());
	
	CSGModel* csgmodel = &makeCube(planeSize, planeSize, planeSize);
	
	csgmodel->translate(0, 0, planeSize/2);
	csgmodel->transform(t);
	//csgmodel->transform(Transform3D<>(point, Rotation3D<>()));
	
	return *csgmodel;
}



//----------------------------------------------------------------------
CSGModel& CSGModel::makeWedge(float angle)
{
	return
		makePlane(Vector3D<>(), Vector3D<>(-sin(angle/2), -cos(angle/2), 0)) *
		makePlane(Vector3D<>(), Vector3D<>(-sin(angle/2), cos(angle/2), 0));
}



//----------------------------------------------------------------------
CSGModel& CSGModel::translate(float x, float y, float z)
{
	Transform3D<> t(Vector3D<>(x, y, z), Rotation3D<>());
	
	transform(t);
	
	return *this;
}



//----------------------------------------------------------------------
CSGModel& CSGModel::translated(float x, float y, float z) const
{	
	CSGModel* result = new CSGModel(*this);
	
	return result->translate(x, y, z);
}



//----------------------------------------------------------------------
CSGModel& CSGModel::rotate(float r, float p, float y)
{
	Transform3D<> t(Vector3D<>(), RPY<>(r, p, y).toRotation3D());
	
	transform(t);
	
	return *this;
}



//----------------------------------------------------------------------
CSGModel& CSGModel::rotated(float r, float p, float y) const
{	
	CSGModel* result = new CSGModel(*this);
	
	return result->rotate(r, p, y);
}



//----------------------------------------------------------------------
CSGModel& CSGModel::transform(const math::Transform3D<>& T)
{
	// convert all of the model vertices to Vector3D & apply transform
	for (unsigned i = 0; i < _model.vertices.size(); ++i) {
		Vector3D<> v(_model.vertices[i].pos.x, _model.vertices[i].pos.y, _model.vertices[i].pos.z);
		
		v = T * v;
		
		_model.vertices[i].pos.x = static_cast<float>(v[0]);
		_model.vertices[i].pos.y = static_cast<float>(v[1]);
		_model.vertices[i].pos.z = static_cast<float>(v[2]);
	}
	
	_needsConversion = true;
	
	return *this;
}



//----------------------------------------------------------------------
CSGModel& CSGModel::transformed(const math::Transform3D<>& T) const
{	
	CSGModel* result = new CSGModel(*this);
	
	return result->transform(T);
}



//----------------------------------------------------------------------
CSGModel& CSGModel::operator+=(const CSGModel& csgmodel)
{
	_model = csgjs_union(_model, csgmodel._model);
	
	_needsConversion = true;
	
	return *this;
}



//----------------------------------------------------------------------
CSGModel& CSGModel::operator+(const CSGModel& csgmodel)
{
	CSGModel* result = new CSGModel(*this);
	
	return *result += csgmodel;
}



//----------------------------------------------------------------------
CSGModel& CSGModel::operator-=(const CSGModel& csgmodel)
{
	_model = csgjs_difference(_model, csgmodel._model);
	
	_needsConversion = true;
	
	return *this;
}



//----------------------------------------------------------------------
CSGModel& CSGModel::operator-(const CSGModel& csgmodel)
{
	CSGModel* result = new CSGModel(*this);
	
	return *result -= csgmodel;
}



//----------------------------------------------------------------------
CSGModel& CSGModel::operator*=(const CSGModel& csgmodel)
{
	_model = csgjs_intersection(_model, csgmodel._model);
	
	_needsConversion = true;
	
	return *this;
}



//----------------------------------------------------------------------
CSGModel& CSGModel::operator*(const CSGModel& csgmodel)
{
	CSGModel* result = new CSGModel(*this);
	
	return *result *= csgmodel;
}



//----------------------------------------------------------------------
CSGModel& CSGModel::operator/=(const CSGModel& csgmodel)
{
	_model = csgjs_difference(csgjs_union(_model, csgmodel._model), csgjs_intersection(_model, csgmodel._model));
	
	_needsConversion = true;
	
	return *this;
}



//----------------------------------------------------------------------
CSGModel& CSGModel::operator/(const CSGModel& csgmodel)
{
	CSGModel* result = new CSGModel(*this);
	
	return *result /= csgmodel;
}



//----------------------------------------------------------------------
TriMesh::Ptr CSGModel::getTriMesh()
{
	// if the model was changed, apply conversion
	if (_needsConversion) _convertToTriMesh();
	
	return _mesh;
}



//----------------------------------------------------------------------
void CSGModel::print() const
{
	cout << *this << endl;
}



//----------------------------------------------------------------------
void CSGModel::saveToStl(const std::string& filename)
{
	// if the model was changed, apply conversion
	if (_needsConversion) _convertToTriMesh();
	
	STLFile::save(*_mesh, filename);
}



//----------------------------------------------------------------------
ostream& rw::csg::operator<<(ostream& stream, const CSGModel& csgmodel)
{
	const csgjs_model& model = csgmodel._model;
	
	stream << "Triangles: " << model.indices.size() / 3 << std::endl;
	
	// output all vertices
	unsigned n = 0;
	for (std::vector<csgjs_vertex>::const_iterator i = model.vertices.begin(); i != model.vertices.end(); ++i) {
		stream << "v" << n++ << ": (" << i->pos.x << ", " << i->pos.y << ", " << i->pos.z << ")" << std::endl;
	}
	
	// output all triangles
	n = 0;
	for (unsigned i = 0; i < (model.indices.size() / 3); ++i) {
		stream << "t" << n++ << ": (" << model.indices[3*i+0] << ", " << model.indices[3*i+1] << ", " << model.indices[3*i+2] << ")" << std::endl;
	}
	
	return stream;
}



//----------------------------------------------------------------------
void CSGModel::_convertToTriMesh()
{
	_mesh = CSGConvert::csgjs_model2TriMesh(_model);
}
