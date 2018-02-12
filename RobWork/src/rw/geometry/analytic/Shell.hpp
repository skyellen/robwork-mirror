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

#ifndef RW_GEOMETRY_ANALYTIC_SHELL_HPP_
#define RW_GEOMETRY_ANALYTIC_SHELL_HPP_

/**
 * @file Shell.hpp
 *
 * \copydoc rw::geometry::Shell
 */

#include <rw/geometry/GeometryData.hpp>
#include <rw/geometry/OBB.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw {
namespace geometry {

class Face;
class GenericFace;

//! @addtogroup geometry

//! @{
/**
 * @brief A shell represents the geometry of an object as a collection of non-connected faces.
 *
 * Each face is composed of a trimmed surface, the edge curves, and vertices that connects the edges.
 * Contrary to the BREP representation, the shell representation holds no information about how each
 * face is connected to the neighbouring faces.
 */
class Shell: public rw::geometry::GeometryData {
public:
    //! @brief Smart pointer type to Shell
    typedef rw::common::Ptr<Shell> Ptr;

    //! @brief Smart pointer type to const Shell
    typedef rw::common::Ptr<const Shell> CPtr;

	//! @brief Constructor.
	Shell();

	//! @brief Destructor.
	virtual ~Shell();

	//! @copydoc GeometryData::getType
	virtual GeometryType getType() const = 0;

	//! @copydoc GeometryData::getTriMesh
	virtual rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true);

	//! @copydoc GeometryData::isConvex
	virtual bool isConvex() = 0;

	//! @copydoc GeometryData::getTriMesh
	virtual rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true) const;

	/**
	 * @brief Get the number of surface patches in this shell.
	 * @return the number of surface patches.
	 */
	virtual std::size_t size() const = 0;

	/**
	 * @brief Get a surface patch.
	 * @param idx [in] index of patch.
	 * @return a copy of the surface patch.
	 */
	inline rw::common::Ptr<const Face> getFace(std::size_t idx) const { return doGetFace(idx); }

	/**
	 * @brief Get a surface patch.
	 * @param idx [in] index of patch.
	 * @param face [out] existing face to copy data into.
	 */
	virtual void getFace(std::size_t idx, GenericFace& face) const;

	/**
	 * @brief Get the minimum and maximum values of the shell in a certain direction.
	 * @param dir [in] the direction to find extremums for.
	 * @return the minimum and maximum as a pair of values.
	 */
	virtual std::pair<double,double> extremums(const rw::math::Vector3D<>& dir) const;

	/**
	 * @brief Create Oriented Bounding Box (OBB) as a bounding volume for the shell.
	 * @return the OBB.
	 */
	virtual OBB<> obb() const;

private:
	virtual rw::common::Ptr<const Face> doGetFace(std::size_t idx) const = 0;
};
//! @}
} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_ANALYTIC_SHELL_HPP_ */
