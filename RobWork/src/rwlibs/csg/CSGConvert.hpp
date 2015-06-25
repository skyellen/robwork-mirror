/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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
 
#ifndef _RWLIBS_CSG_CSGCCONVERT_HPP
#define _RWLIBS_CSG_CSGCCONVERT_HPP



#include <rw/geometry/TriMesh.hpp>

struct csgjs_model;



namespace rwlibs {
namespace csg {
	
/**
 * Provides conversion methods between TriMesh class and csgjs_model class.
 */
class CSGConvert
{
public:
	/** Converts TriMesh to csgjs_model */
	static rw::common::Ptr<csgjs_model> TriMesh2csgjs_model(const rw::geometry::TriMesh& mesh);
	
	/** Converts csgjs_model to TriMesh */
	static rw::geometry::TriMesh::Ptr csgjs_model2TriMesh(const csgjs_model& model);
};

} /* csg */
} /* rwlibs */

#endif
