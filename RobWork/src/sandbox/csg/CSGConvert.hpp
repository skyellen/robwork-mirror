/**
 * @file CSGConvert.hpp
 * 
 * @brief contains conversion methods between RobWork TriMesh
 * representation of geometry and the one of csgjs library
 * 
 * @author Adam Wolniakowski
 */
 
#pragma once



#include <rw/geometry/TriMesh.hpp>

struct csgjs_model;



namespace rw {
	namespace csg {
		/**
		 * provides conversion methods between TriMesh class and csgjs_model class
		 */
		class CSGConvert
		{
			public:
				/** convert TriMesh to csgjs_model */
				static csgjs_model* TriMesh2csgjs_model(const geometry::TriMesh& mesh);
				
				/** convert csgjs_model to TriMesh */
				static geometry::TriMesh::Ptr csgjs_model2TriMesh(const csgjs_model& model);
		};
	} // csg
} //rw
