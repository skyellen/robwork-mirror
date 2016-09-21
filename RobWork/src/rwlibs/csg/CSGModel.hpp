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
 
#ifndef _RWLIBS_CSG_CSGMODEL_HPP
#define _RWLIBS_CSG_CSGMODEL_HPP

#include <rw/math/Transform3D.hpp>
#include <rw/geometry/TriMesh.hpp>

#define CSGJS_HEADER_ONLY
#include <csgjs/csgjs.cpp>



namespace rwlibs {
namespace csg {
	
/**
 * @brief A CSG (Constructive Solid Geometry) model representation.
 * 
 * Defines operators to perform addition, difference, and intersection operations on solids,
 * as well as utility functions to construct geometry based on primitives, translate
 * and rotate models, and convert them to and from RobWork represntations.
 */
class CSGModel
{
public:
	//! @brief Smart pointer type for CSGModel.
	typedef rw::common::Ptr<CSGModel> Ptr;
	
public:
	//! @brief Constructor.
	CSGModel();
	
	//! @brief Destructor.
	virtual ~CSGModel() {}
	
	/**
	 * @brief Copy constructor.
	 * @param csgmodel [in] other CSGModel to copy.
	 */
	CSGModel(const CSGModel& csgmodel);
	
	/** @brief Constructs CSGModel from TriMesh. */
	CSGModel(const rw::geometry::TriMesh& trimesh);
	
	/* TRANSFORMATIONS */	
	/** @brief Translates the model. */
	void translate(float x, float y, float z);
	
	/** @brief Rotates the model. */
	void rotate(float r, float p, float y);
	
	/** @brief Applies RobWork transformation to the model. */
	void transform(const rw::math::Transform3D<>& T);
	
	/* OPERATIONS */
	/**
	 * @brief Adds a volume.
	 */
	void add(CSGModel::Ptr model); 
	
	/**
	 * @brief Subtracts a volume.
	 */
	void subtract(CSGModel::Ptr model);
	
	/**
	 * @brief Intersects volumes.
	 */
	void intersect(CSGModel::Ptr model);
	
	/* UTILITIES */
	/** @brief Returns RobWork geometry representation. */
	rw::geometry::TriMesh::Ptr getTriMesh();

	/** @brief Saves the CSGModel in Stl format. */
	void saveToStl(const std::string& filename);
	
private:
	/** @brief Converts internal csgjs geometry representation to TriMesh. */
	void _convertToTriMesh();
	
	bool _needsConversion; // is it neccessary to convert to TriMesh?
	
	rw::common::Ptr<csgjs_model> _model; // csgjs library geometry representation
	rw::geometry::TriMesh::Ptr _mesh; // RobWork geometry representation
};

} /* csg */
} /* rwlibs */

#endif
