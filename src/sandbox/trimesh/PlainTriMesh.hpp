#ifndef PLAINTRIMESH_HPP_
#define PLAINTRIMESH_HPP_

#include "TriMesh.hpp"

template <class T=double, TriType TRI=N0>
class PlainTriMesh: public TriMesh<T> {
private:
	std::vector<Triangle<T, TRI> > _triangles;
	
public:
	//typedef TRI value_type;
	
	PlainTriMesh(){}
	
	void add(const Triangle<T,TRI>& triangle){
		_triangles.push_back(triangle);
	}
	
	std::vector<Triangle<T,TRI> >& getTriangles(){
		return _triangles;
	}
	
	// Inherited from TriMesh
	Triangle<T,N0> getTriangle(size_t idx) const {
		return _triangles[idx];
	}
	
	size_t getSize() const {
		return _triangles.size();
	}

	
};



#endif /*TRIMESH_HPP_*/
