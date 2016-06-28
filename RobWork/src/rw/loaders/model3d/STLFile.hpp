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


#ifndef RW_GEOMETRY_STLFILE_HPP_
#define RW_GEOMETRY_STLFILE_HPP_

#include <rw/geometry/PlainTriMesh.hpp>

namespace rw { namespace geometry { class TriMesh; } }

namespace rw {
namespace loaders {
	//! @addtogroup geometry
	// @{

	/**
	 * @brief static methods for reading and writing geometry to and from
	 * STL files.
	 */
	class STLFile {
	public:

		/**
		 * @brief creates a new ASCII STL file with path+name given by \b filename.
		 * The face data is taken from a TriMesh interface.
		 * @param mesh [in] the mesh that should be written to the STL file.
		 * @param filename [in] the name of the file for which to write to.
		 */
		static void save(const rw::geometry::TriMesh& mesh, const std::string& filename);

		/**
		 * @brief reads a STL file with name \b filename into a plain
		 * triangle mesh.
		 * @param filename [in] the name of the file
		 * @return triangle mesh if successfull, NULL otherwise.
		 */
		static rw::geometry::PlainTriMeshN1F::Ptr load(const std::string& filename);

	};

	// @}
}
}

#endif /*STLFILE_HPP_*/
