/********************************************************************************
 * Copyright 2018 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_GEOMETRY_ANALYTIC_FACE_HPP_
#define RW_GEOMETRY_ANALYTIC_FACE_HPP_

/**
 * @file Face.hpp
 *
 * \copydoc rw::geometry::Face
 */

#include <rw/common/Ptr.hpp>
#include <rw/geometry/OBB.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>

namespace rw {
namespace geometry {

class Curve;
class Surface;
class TriMesh;

//! @addtogroup geometry

//! @{
/**
 * @brief Abstract interface for geometric faces.
 *
 * A face consist of a surface and curves that form the boundary of the face.
 *
 * For all faces there must be the same number of vertices and curves.
 * The order of vertices and curves are ordered such that a curve at a certain index will have a corresponding start vertex at the same vertex index.
 */
class Face {
public:
    //! @brief Smart pointer type to Face
    typedef rw::common::Ptr<Face> Ptr;

    //! @brief Smart pointer type to const Face
    typedef rw::common::Ptr<const Face> CPtr;

    //! @brief Constructor.
	Face();

	//! @brief Destructor.
	virtual ~Face();

	/**
	 * @brief Get the surface of the face.
	 * @return a reference to the surface data.
	 */
	virtual const Surface& surface() const = 0;

	/**
	 * @brief Get the number of curves in the face.
	 * @return the number of curves.
	 */
	virtual std::size_t curveCount() const = 0;

	/**
	 * @brief Get a curve of the face.
	 * @param i [in] the curve index, which should be less than the number returned by curveCount().
	 * @return a reference to the curve data.
	 */
	virtual const Curve& getCurve(std::size_t i) const = 0;

	/**
	 * @brief Get the vertices of the face.
	 * @return a reference to the vertex vector.
	 */
	virtual const std::vector<rw::math::Vector3D<> >& vertices() const = 0;

	/**
	 * @brief Transform the face.
	 * @param T [in] transform.
	 */
	virtual void transform(const rw::math::Transform3D<>& T) = 0;

	/**
	 * @brief Translation of face.
	 * @param P [in] translation vector.
	 */
	virtual void transform(const rw::math::Vector3D<>& P) = 0;

	/**
	 * @brief Create a TriMesh representation of the face.
	 *
	 * This function relies on the resolution set with setMeshResolution.
	 * The resolution is passed on to Curve::discretizeAdaptive and Surface::setDiscretizationResolution.
	 *
	 * @param forceCopy [in] (not currently used in default implementation)
	 * @return a new TriMesh.
	 */
	virtual rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true) const;

	/**
	 * @brief Find the extent of the surface along a specific direction.
	 * @param dir [in] a normalized direction vector.
	 * @return the minimum and maximum values along the given direction.
	 */
	virtual std::pair<double,double> extremums(const rw::math::Vector3D<>& dir) const;

	/**
	 * @brief Create Oriented Bounding Box.
	 *
	 * The default implementation forms a TriMesh in order to estimate the directions for the OBB.
	 *
	 * @return an OBB around the Face.
	 */
	virtual OBB<> obb();

	/**
	 * @brief Set the resolution used for discretization in the getTriMesh and faceTriMesh functions.
	 *
	 * The meaning of this parameter depends on the type of surface.
	 *
	 * @param resolution [in] the resolution parameter.
	 */
	void setMeshResolution(double resolution) { _resolution = resolution; }

protected:
	//! @brief Resolution used for discretization into triangle meshes.
	double _resolution;
};
//! @}
} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_ANALYTIC_FACE_HPP_ */
