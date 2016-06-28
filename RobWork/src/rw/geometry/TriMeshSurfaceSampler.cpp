#include "TriMeshSurfaceSampler.hpp"

#include <rw/math/Math.hpp>

#include <boost/foreach.hpp>

using namespace rw::geometry;

TriMeshSurfaceSampler::TriMeshSurfaceSampler(rw::geometry::Geometry::Ptr geom):
        _minD(0.02),
        _maxD(0.02),
        _genRandomRotation(true),
        _filterByDirection(false),
        _genRandomPostion(true)
{
	add(geom);
}

TriMeshSurfaceSampler::TriMeshSurfaceSampler(const std::vector<rw::geometry::Geometry::Ptr>& geoms):
        _minD(0.02),
        _maxD(0.02),
        _genRandomRotation(true),
        _filterByDirection(false),
        _genRandomPostion(true)
{
	BOOST_FOREACH(Geometry::Ptr geom, geoms){
		add(geom);
	}
}

TriMeshSurfaceSampler::~TriMeshSurfaceSampler(){}

void TriMeshSurfaceSampler::add(rw::geometry::Geometry::Ptr geom){
	rw::geometry::TriMesh::Ptr mesh = geom->getGeometryData()->getTriMesh();
	add(mesh);
}

void TriMeshSurfaceSampler::add(rw::geometry::TriMesh::Ptr mesh){
	// add mesh to surface area vector
	int start_idx = (int)_surfaceArea.size();
	_surfaceArea.resize( start_idx + mesh->size());
	_sAreaSum = 0;
	if(start_idx>0)
		_sAreaSum = _surfaceArea[start_idx-1];

	// run through mesh and create the search
	for(size_t i=0; i<mesh->size(); i++){
		rw::geometry::Triangle<> tri = mesh->getTriangle(i);
		// calculate triangle area
		_sAreaSum += tri.calcArea();
		_surfaceArea[i] = _sAreaSum;
	}

	_meshes.push_back(mesh);
	_surfaceAreaMesh.push_back( start_idx );
}

rw::geometry::Triangle<> TriMeshSurfaceSampler::getTriangle(const int idx){
	// first find the geometry in which the triangle is located
	if(_meshes.size()==1){
		return _meshes[0]->getTriangle(idx);
	} else if(_meshes.size()==2) {
		if(idx<_surfaceAreaMesh[0])
			return _meshes[0]->getTriangle(idx);
		else
			return _meshes[1]->getTriangle(idx-_surfaceAreaMesh[0]);
	}
	// find the geometry in which the triangle with idx exists
	int lower=0,upper=(int)_surfaceAreaMesh.size();

	do {
		int split = lower+(upper-lower)/2;
		if(idx<_surfaceAreaMesh[split]){
			upper = split;
		} else {
			lower = split+1;
		}
	} while(upper!=lower);
	return _meshes[lower]->getTriangle(idx);
}

rw::math::Vector3D<> TriMeshSurfaceSampler::samplePoint(){
	using namespace rw::math;
	double rnum = Math::ran(0.0, _sAreaSum);
	int triIds = binSearchRec(rnum, 0, _surfaceArea.size()-1);
	rw::geometry::Triangle<> tri = getTriangle(triIds);

	// random sample the triangle
	double b0 = Math::ran();
	double b1 = ( 1.0f - b0 ) * Math::ran();
	double b2 = 1 - b0 - b1;

	return tri[0] * b0 + tri[1] * b1 + tri[2] * b2;
}


rw::math::Transform3D<> TriMeshSurfaceSampler::sample(){
	using namespace rw::math;
	Transform3D<> target;
	do{
		double rnum = rw::math::Math::ran(0.0, _sAreaSum);
		int triIds = binSearchRec(rnum, 0, _surfaceArea.size()-1);
		rw::geometry::Triangle<> tri = getTriangle(triIds);

		// random sample the triangle
		double b0 = Math::ran();
		double b1 = ( 1.0f - b0 ) * Math::ran();
		double b2 = 1 - b0 - b1;

		Vector3D<> position = tri[0] * b0 + tri[1] * b1 + tri[2] * b2;

		// and sample the orientation
		//EAA<> eaa(Vector3D<>::z(), -tri.calcFaceNormal());
		if(_genRandomPostion){
			target = Transform3D<>( position, Math::ranRotation3D<double>());
			target.P() -= (target.R()*Vector3D<>::z())*Math::ran(_minD,_maxD);
		} else {
			target.P() = position;
			// align z-axis of rotation with triangle normal
			EAA<> eaa( Vector3D<>::z(), tri.calcFaceNormal() );
			target.R() = eaa.toRotation3D();
		}

		if(_genRandomRotation){
			target.R() = Math::ranRotation3D<double>();
		}
		if(!_filterByDirection)
			return target;
	} while ( dot(_direction,target.R()*Vector3D<>::z())>0);
	return target;
}

void TriMeshSurfaceSampler::setBoundsD(double minD, double maxD){
	if(maxD<=minD)
		RW_THROW("Min bound must be smaller than max bound! [" << minD << ";" << maxD << "]");
	_minD = minD;
	_maxD = maxD;
}

void TriMeshSurfaceSampler::setRandomRotationEnabled(bool enabled){
	_genRandomRotation = enabled;
}

void TriMeshSurfaceSampler::setRandomPositionEnabled(bool enabled){
	_genRandomPostion = enabled;
}

void TriMeshSurfaceSampler::setZAxisDirectionEnabled(bool enabled) { _filterByDirection = enabled; }

void TriMeshSurfaceSampler::setZAxisDirection(const rw::math::Vector3D<>& dir){
	_direction = dir;
	setZAxisDirectionEnabled(true);
}

rw::geometry::TriMesh::Ptr TriMeshSurfaceSampler::getMesh(){ return _meshes[0]; }
