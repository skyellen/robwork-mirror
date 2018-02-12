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

#ifndef RW_GEOMETRY_ANALYTIC_SURFACE_HPP_
#define RW_GEOMETRY_ANALYTIC_SURFACE_HPP_

/**
 * @file Surface.hpp
 *
 * \copydoc rw::geometry::Surface
 */

#include <rw/common/Ptr.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>

namespace rw {
namespace geometry {

class TriMesh;

//! @addtogroup geometry

//! @{
/**
 * @brief Surface is an abstract representation of a smooth surface geometry.
 *
 * The interface provides functions for affine transformations, such as scaling, rotation and translation.
 * In case of a trimmed surface, it is also possible to make a discretization of the surface to triangle mesh.
 */
class Surface {
public:
	//! @brief Smart pointer type for Surface.
	typedef rw::common::Ptr<Surface> Ptr;
	//! @brief Smart pointer type for const Surface.
	typedef rw::common::Ptr<const Surface> CPtr;

	//! @brief Constructor.
	Surface() {}

	//! @brief Destructor.
	virtual ~Surface() {}

	/**
	 * @brief Move the surface.
	 * @param T [in] the transform to the new surface.
	 * @return pointer to a new surface.
	 * @see #transform(const rw::math::Vector3D<>&) const if there is no rotation.
	 * This will preserve some nice properties for certain types of surfaces.
	 */
	inline Surface::Ptr transform(const rw::math::Transform3D<>& T) const { return doTransformSurface(T); }

	/**
	 * @brief Move the surface without rotation.
	 *
	 * If there is no rotation, this function is better to use than #transform(const rw::math::Transform3D<>&) const
	 * for some surfaces. This is because certain properties can be preserved.
	 *
	 * @param P [in] the translation vector to the new surface.
	 * @return pointer to a new surface.
	 */
	inline Surface::Ptr transform(const rw::math::Vector3D<>& P) const { return doTransformSurface(P); }

	/**
	 * @brief Get a scaled version of the surface.
	 * @param factor [in] the factor to scale with.
	 * @return a new scaled surface.
	 */
	inline Surface::Ptr scale(double factor) const { return doScaleSurface(factor); }

	/**
	 * @brief Clone the surface.
	 * @return pointer to copy of surface.
	 */
	inline Surface::Ptr clone() const { return doCloneSurface(); }

	/**
	 * @brief Find the extent of the surface along a specific direction.
	 *
	 * If the surface has no lower bound, the value -%std::numeric_limits\<double\>::%max() can be returned
	 * to indicate that the surface has unbounded minimum value in the given \b direction.
	 *
	 * If the surface has no upper bound, the value %std::numeric_limits\<double\>::%max() can be returned
	 * to indicate that the surface has unbounded maximum value in the given \b direction.
	 *
	 * @param direction [in] a normalized direction vector.
	 * @return the minimum and maximum values along the given direction.
	 * @note This function does not take trimming conditions into account.
	 * For trimmed surfaces, create a Face with the boundary curves and use Face::extremums.
	 */
	virtual std::pair<double,double> extremums(const rw::math::Vector3D<>& direction) const = 0;

	/**
	 * @brief Discretize the surface into a triangle mesh representation.
	 *
	 * If the border of a trimmed surface must fit with other surface triangulations,
	 * a discretized \b border must be given to this triangulation function.
	 *
	 * If the same border points are used for different surfaces for their common edges, the triangulations will fit together.
	 *
	 * @param border [in] (optional) an ordered list of points on the surface, that forms the border of the patch to triangulate.
	 * @return a new TriMesh.
	 */
	virtual rw::common::Ptr<TriMesh> getTriMesh(const std::vector<rw::math::Vector3D<> >& border = std::vector<rw::math::Vector3D<> >()) const = 0;

	/**
	 * @brief Set the resolution used for discretization in the getTriMesh function.
	 *
	 * The meaning of this parameter depends on the type of surface.
	 *
	 * @param resolution [in] the resolution parameter.
	 */
	virtual void setDiscretizationResolution(double resolution) = 0;

private:
	virtual Surface::Ptr doTransformSurface(const rw::math::Transform3D<>& T) const = 0;
	virtual Surface::Ptr doTransformSurface(const rw::math::Vector3D<>& P) const = 0;
	virtual Surface::Ptr doScaleSurface(double factor) const = 0;
	virtual Surface::Ptr doCloneSurface() const = 0;
};
//! @}
} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_ANALYTIC_SURFACE_HPP_ */
