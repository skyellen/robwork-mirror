#include "IndexedTriMesh.hpp"

using namespace rw::geometry;

template class IndexedTriMesh<double,N0>;
template class IndexedTriMesh<double,N1>;
template class IndexedTriMesh<double,N3>;

template class IndexedTriMesh<float,N0>;
template class IndexedTriMesh<float,N1>;
template class IndexedTriMesh<float,N3>;
