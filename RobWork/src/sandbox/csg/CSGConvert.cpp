#include "CSGConvert.hpp"

#include <rw/rw.hpp>
#include <rw/geometry/TriMesh.hpp>
#define CSGJS_HEADER_ONLY
#include "csgjs.cpp"


using namespace rw;
using namespace rw::common;
using namespace rw::math;
using namespace rw::geometry;
using namespace rw::csg;



csgjs_model* CSGConvert::TriMesh2csgjs_model(const geometry::TriMesh& mesh)
{
	csgjs_model* model = new csgjs_model;
	
	// iterate over all triangles
	for (unsigned i = 0; i < mesh.getSize(); ++i) {
		const Triangle<double>& t = mesh.getTriangle(i);
		
		
		// get triangle normal
		const Vector3D<float> tnormal = cast<float>(t.calcFaceNormal());
		
		// create vertices
		Vector3D<float> tv;
		csgjs_vertex v;
		v.normal = csgjs_vector(tnormal[0], tnormal[1], tnormal[2]);
		
		tv = cast<float>(t[0]);
		v.pos = csgjs_vector(tv[0], tv[1], tv[2]);
		model->vertices.push_back(v);
		model->indices.push_back(3*i+0);
		
		tv = cast<float>(t[1]);
		v.pos = csgjs_vector(tv[0], tv[1], tv[2]);
		model->vertices.push_back(v);
		model->indices.push_back(3*i+1);
		
		tv = cast<float>(t[2]);
		v.pos = csgjs_vector(tv[0], tv[1], tv[2]);
		model->vertices.push_back(v);
		model->indices.push_back(3*i+2);
	}
	
	return model;
}



TriMesh::Ptr CSGConvert::csgjs_model2TriMesh(const csgjs_model& model)
{
	PlainTriMeshD* mesh = new PlainTriMeshD;
	
	// loop over all triangles
	for (unsigned i = 0; i < (model.indices.size() / 3); ++i) {
		csgjs_vector v1 = model.vertices[model.indices[3*i+0]].pos;
		csgjs_vector v2 = model.vertices[model.indices[3*i+1]].pos;
		csgjs_vector v3 = model.vertices[model.indices[3*i+2]].pos;
			
		Vector3D<double> p1(v1.x, v1.y, v1.z);
		Vector3D<double> p2(v2.x, v2.y, v2.z);
		Vector3D<double> p3(v3.x, v3.y, v3.z);
		
		mesh->add(Triangle<double>(p1, p2, p3));
	}
	
	return ownedPtr(mesh);
}



/*std::ostream& rw::csg::operator<<(std::ostream& stream, const csgjs_model& model)
{
	stream << "Number of triangles: " << model.indices.size() / 3 << std::endl;
	
	// output all vertices
	int n = 0;
	for (std::vector<csgjs_vertex>::const_iterator i = model.vertices.begin(); i != model.vertices.end(); ++i) {
		stream << "v" << n++ << ": (" << i->pos.x << ", " << i->pos.y << ", " << i->pos.z << ")" << std::endl;
	}
	
	// output all triangles
	n = 0;
	for (unsigned i = 0; i < (model.indices.size() / 3); ++i) {
		stream << "t" << n++ << ": (" << model.indices[3*i+0] << ", " << model.indices[3*i+1] << ", " << model.indices[3*i+2] << ")" << std::endl;
	}
	
	return stream;
}*/
