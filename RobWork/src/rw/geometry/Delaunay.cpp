/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "Delaunay.hpp"

#if defined(__cplusplus)
extern "C" {
#include <libqhull_r/libqhull_r.h>
}
#endif

using rw::common::ownedPtr;
using namespace rw::math;
using namespace rw::geometry;

namespace {
void build(size_t dim,
		double *coords,
		size_t nrCoords,
		std::vector<int>& vertIdxs,
		std::vector<int>& faceIdxs)
{
	vertIdxs.clear();
	faceIdxs.clear();

	std::vector<int>& result = vertIdxs;

	int curlong, totlong; /* used !qh_NOmem */
    int exitcode;
    boolT ismalloc = false;
	qhT qh;
	qhT * qhT_pointer = &qh; // allocate memory for qhull

	//qh_init_A (qhT_pointer, stdin, stdout, stderr, 0, NULL);
    char flags[] = "qhull d Qbb Qt";
    //char flags[] = "qhull Pp QJ";
	//qh_initflags(qhT_pointer, flags);

	// According to doc. in libqhull_r/user_r.c, the qhull memory should be cleared:
	qh_zero(qhT_pointer, stderr);

	// Then the object is constructed.
	exitcode = qh_new_qhull(qhT_pointer, (int)dim, (int)nrCoords, coords, ismalloc, flags, NULL, stderr);
	//qh_setdelaunay(qhT_pointer, (int)dim, (int)nrCoords, coords);

	// Loop through all vertices,
	vertexT *vertex;//, **vertexp;
	for (vertex=qhT_pointer->vertex_list;vertex && vertex->next;vertex= vertex->next) {
		int vertexIdx = qh_pointid(qhT_pointer,vertex->point);
		result.push_back(vertexIdx);
	}

	if (!exitcode) {
		// For all facets:
		for (facetT* facet = qhT_pointer->facet_list; facet && facet->next; facet = facet->next) {
			if (!facet->upperdelaunay) {
				int vertex_n, vertex_i;
				FOREACHvertex_i_(qhT_pointer,facet->vertices) {
					int vertexIdx = qh_pointid(qhT_pointer,vertex->point);
					faceIdxs.push_back(vertexIdx);
				}
			}
		}
	} else {
		std::cout << "Not Exit Code!";
	}

	if (qhT_pointer->VERIFYoutput && !qhT_pointer->FORCEoutput && !qhT_pointer->STOPpoint && !qhT_pointer->STOPcone)
		qh_check_points(qhT_pointer);

	qh_freeqhull(qhT_pointer, False);
	qh_memfreeshort (qhT_pointer,&curlong, &totlong);
	if (curlong || totlong)
		fprintf (stderr, "qdelaunay internal warning (main): did not free %d bytes of long memory (%d pieces)\n",totlong, curlong);
}

template<typename S>
typename IndexedTriMeshN0<double,S>::Ptr makeMesh(const std::vector<Vector2D<> >& hullVertices, const std::vector<int>& faceIdxs, const std::vector<double>& values) {
	typedef IndexedTriMeshN0<double,S> Mesh;
	typedef typename Mesh::VertexArray VertexArray;
	typedef typename Mesh::TriangleArray TriangleArray;

	// Add vertices
	const rw::common::Ptr<VertexArray> meshV = ownedPtr(new VertexArray(hullVertices.size()));
	for(size_t i = 0; i < hullVertices.size(); i++) {
		(*meshV)[i] = Vector3D<>(hullVertices[i][0], hullVertices[i][1], (values.size() == hullVertices.size()) ? values[i] : 0);
	}

	// Add triangles
	const rw::common::Ptr<TriangleArray> meshTri = ownedPtr(new TriangleArray(faceIdxs.size()/3));
	for(size_t i = 0; i < faceIdxs.size()/3; i++) {
		const typename Mesh::tri_type tempTri(faceIdxs[i*3+0], faceIdxs[i*3+1], faceIdxs[i*3+2]);
		const Vector3D<>& v1 = (*meshV)[tempTri[0]];
		const Vector3D<>& v2 = (*meshV)[tempTri[1]];
		const Vector3D<>& v3 = (*meshV)[tempTri[2]];

		// Make sure the vertices order is correct according to right-hand rule. Otherwise flip v2 and v3.
		const double dotProduct = dot(cross(v2-v1,v3-v2),Vector3D<>::z());
		if (std::fabs(dotProduct) > 0.001) {
			if (dotProduct < 0.0) {
				(*meshTri)[i] = typename Mesh::tri_type(tempTri[0],tempTri[2],tempTri[1]);
			} else {
				(*meshTri)[i] = tempTri;
			}
		}
	}

	const typename Mesh::Ptr mesh = ownedPtr(new Mesh(meshV,meshTri));
	return mesh;
}
}

Delaunay::Delaunay() {
}

Delaunay::~Delaunay() {
}

IndexedTriMesh<>::Ptr Delaunay::triangulate(const std::vector<Vector2D<> >& vertices, const std::vector<double>& values) {
	if (vertices.size() == 3) {
		std::vector<int> faceIdxs(3);
		faceIdxs[0] = 0;
		faceIdxs[1] = 1;
		faceIdxs[2] = 2;
		return makeMesh<uint8_t>(vertices,faceIdxs,values);
	}

    // convert the vertice array to an array of double
    double *vertArray = new double[vertices.size()*2];
    // copy all data into the vertArray
    for(size_t i = 0; i < vertices.size(); i++) {
        const Vector2D<> &v = vertices[i];
        vertArray[i*2+0] = v[0];
        vertArray[i*2+1] = v[1];
    }
    // Build delaunay triangulation
    std::vector<Vector2D<> > triangVertices;
    std::vector<int> vertiIdxs;
    std::vector<int> faceIdxs;
    build(2, vertArray, vertices.size(), vertiIdxs, faceIdxs);
    delete[] vertArray;

    std::vector<int> vertIdxMap(vertices.size());
    triangVertices.resize(vertiIdxs.size());
    for(size_t i = 0; i < vertiIdxs.size(); i++) {
    	triangVertices[i] = vertices[vertiIdxs[i]];
        vertIdxMap[vertiIdxs[i]] = (int)i;
    }
    for(size_t i = 0; i < faceIdxs.size(); i++) {
        const int tmp = faceIdxs[i];
        faceIdxs[i] = vertIdxMap[ tmp ];
    }

    // Create the mesh
    if (vertices.size () <= 256) {
    	return makeMesh<uint8_t>(triangVertices,faceIdxs,values);
    } else if (vertices.size () <= 65536) {
        return makeMesh<uint16_t>(triangVertices,faceIdxs,values);
    } else if (vertices.size () <= 4294967296) {
        return makeMesh<uint32_t>(triangVertices,faceIdxs,values);
    } else {
        return makeMesh<uint64_t>(triangVertices,faceIdxs,values);
    }
}
