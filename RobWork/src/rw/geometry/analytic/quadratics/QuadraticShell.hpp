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

#ifndef RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICSHELL_HPP_
#define RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICSHELL_HPP_

/**
 * @file QuadraticShell.hpp
 *
 * \copydoc rw::geometry::QuadraticShell
 */

#include <rw/geometry/analytic/Shell.hpp>
#include <rw/geometry/OBB.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw {
namespace geometry {

class QuadraticFace;

//! @addtogroup geometry

//! @{
/**
 * @brief A collection of Quadratic surface patches, that together form a shell.
 */
class QuadraticShell: public Shell {
public:
    //! @brief Smart pointer type to QuadraticShell
    typedef rw::common::Ptr<QuadraticShell> Ptr;

	//! @brief Smart pointer type for a const QuadraticShell.
	typedef rw::common::Ptr<const QuadraticShell> CPtr;

    //! @brief Constructor.
	QuadraticShell(): _resolution(10) {}

	//! @brief Destructor.
	virtual ~QuadraticShell() {}

	//! @copydoc Shell::getType
	virtual GeometryType getType() const { return GeometryData::Quadratic; }

	//! @copydoc Shell::isConvex
	virtual bool isConvex() = 0;

	//! @copydoc Shell::size
	virtual std::size_t size() const = 0;

	//! @copydoc Shell::getFace
	virtual rw::common::Ptr<const QuadraticFace> getFace(std::size_t idx) const = 0;

	/**
	 * @brief Get a surface patch.
	 * @param idx [in] index of the patch.
	 * @param dst [out] an existing face to write data to.
	 */
	virtual void getFace(std::size_t idx, QuadraticFace& dst) const = 0;

	/**
	 * @brief Set the resolution used for discretization in the getTriMesh and faceTriMesh functions.
	 *
	 * The meaning of this parameter depends on the type of surface.
	 *
	 * @param resolution [in] the resolution parameter.
	 */
	void setMeshResolution(double resolution) { _resolution = resolution; }

private:
	virtual rw::common::Ptr<const Face> doGetFace(std::size_t idx) const;

protected:
	//! @brief Resolution to use for discretization into triangle mesh.
	double _resolution;
};
//! @}
} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICSHELL_HPP_ */
