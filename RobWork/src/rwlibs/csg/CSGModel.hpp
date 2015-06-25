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

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/geometry/TriMesh.hpp>

#include <iostream>

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
	typedef rw::common::Ptr<CSGModel> Ptr;
	
public:
	CSGModel();
	
	virtual ~CSGModel() {}
	
	CSGModel(const CSGModel& csgmodel);
	
	/** @brief Constructs CSGModel from TriMesh. */
	CSGModel(const rw::geometry::TriMesh& trimesh);
	
	/* TRANSFORMATIONS */	
	/** @brief Translates the csgmodel. */
	CSGModel& translate(float x, float y, float z);
	
	/** @brief Returns translated copy of the model. */
	CSGModel translated(float x, float y, float z) const;
	
	/** @brief Rotates the csgmodel. */
	CSGModel& rotate(float r, float p, float y);
	
	/** @brief Returns rotated copy of the model. */
	CSGModel rotated(float r, float p, float y) const;
	
	/** @brief Applies RobWork transformation. */
	CSGModel& transform(const rw::math::Transform3D<>& T);
	
	/** @brief Returns copy of the model after transformation. */
	CSGModel transformed(const rw::math::Transform3D<>& T) const;
	
	/* OPERATIONS */
	//! Performs addition of models.
	CSGModel& operator+=(const CSGModel& csgmodel);
	
	//! Performs addition of models.
	CSGModel operator+(const CSGModel& csgmodel);
	
	//! Performs subtraction of models.
	CSGModel& operator-=(const CSGModel& csgmodel);
	
	//! Performs subtraction of models.
	CSGModel operator-(const CSGModel& csgmodel);
	
	/**
	 * @brief Performs intersection of models.
	 * @return Returns a model consisting of volume common to both models.
	 */
	CSGModel& operator*=(const CSGModel& csgmodel);
	
	/**
	 * @brief Performs intersection of models.
	 * @return Returns a model consisting of volume common to both models.
	 */
	CSGModel operator*(const CSGModel& csgmodel);
	
	/**
	 * @brief Performs XOR operation on models.
	 * @return Returns a model consisting of volume which is not shared by both models.
	 */
	CSGModel& operator/=(const CSGModel& csgmodel);
	
	/**
	 * @brief Performs XOR operation on models.
	 * @return Returns a model consisting of volume which is not shared by both models.
	 */
	CSGModel operator/(const CSGModel& csgmodel);
	
	/** @brief Returns RobWork geometry representation. */
	rw::geometry::TriMesh::Ptr getTriMesh();
	
	/** @brief Print the CSGModel */
	void print() const;
	
	/** @brief Saves the CSGModel in Stl format. */
	void saveToStl(const std::string& filename);
	
	friend std::ostream& operator<<(std::ostream& stream, const CSGModel& csgmodel);
	
private:
	/** @brief Converts internal csgjs geometry representation to TriMesh. */
	void _convertToTriMesh();
	
	bool _needsConversion; // is it neccessary to convert to TriMesh?
	
	csgjs_model _model; // csgjs library geometry representation
	rw::geometry::TriMesh::Ptr _mesh; // RobWork geometry representation
};

std::ostream& operator<<(std::ostream& stream, const CSGModel& csgmodel);

} /* csg */
} /* rwlibs */

#endif
