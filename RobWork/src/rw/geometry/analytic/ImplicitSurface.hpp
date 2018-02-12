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

#ifndef RW_GEOMETRY_ANALYTIC_IMPLICITSURFACE_HPP_
#define RW_GEOMETRY_ANALYTIC_IMPLICITSURFACE_HPP_

/**
 * @file ImplicitSurface.hpp
 *
 * \copydoc rw::geometry::ImplicitSurface
 */

#include "Surface.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw {
namespace geometry {
//! @addtogroup geometry

//! @{
/**
 * @brief Interface for implicit surfaces. An implicit surface is given by an expression of the form \f$ F(\mathbf{x})=0, \mathbf{x} \in \mathbb{R}^3\f$.
 */
class ImplicitSurface: public Surface {
public:
	//! @brief Smart pointer type for ImplicitSurface
	typedef rw::common::Ptr<ImplicitSurface> Ptr;

	//! @brief Smart pointer type for const ImplicitSurface
	typedef rw::common::Ptr<const ImplicitSurface> CPtr;

	//! @brief Constructor.
	ImplicitSurface() {}

	//! @brief Destructor.
	virtual ~ImplicitSurface() {}

	//! @copydoc Surface::transform(const rw::math::Transform3D<>&) const
	inline ImplicitSurface::Ptr transform(const rw::math::Transform3D<>& T) const { return doTransformImplicitSurface(T); }

	//! @copydoc Surface::transform(const rw::math::Vector3D<>&) const
	inline ImplicitSurface::Ptr transform(const rw::math::Vector3D<>& P) const { return doTransformImplicitSurface(P); }

	//! @copydoc Surface::scale
	inline ImplicitSurface::Ptr scale(double factor) const { return doScaleImplicitSurface(factor); }

	//! @copydoc Surface::clone
	inline ImplicitSurface::Ptr clone() const { return doCloneImplicitSurface(); }

	//! @copydoc Surface::extremums
	virtual std::pair<double,double> extremums(const rw::math::Vector3D<>& direction) const = 0;

	//! @copydoc Surface::getTriMesh
	virtual rw::common::Ptr<TriMesh> getTriMesh(const std::vector<rw::math::Vector3D<> >& border = std::vector<rw::math::Vector3D<> >()) const = 0;

	//! @copydoc Surface::setDiscretizationResolution
	virtual void setDiscretizationResolution(double resolution) = 0;

	/**
	 * @brief Evaluate the implicit function, \f$ F(\mathbf{x}) \f$, for the surface.
	 * @param x [in] the point to evaluate.
	 * @return the value of the implicit function. If smaller than zero, \b x lies inside the surface. If larger than zero, \b x lies outside the surface.
	 */
	virtual double operator()(const rw::math::Vector3D<>& x) const = 0;

	/**
	 * @brief Check if point, \b P, on surface lies inside the trimming region.
	 * @param P [in] the point to check.
	 * @return true if the points lies inside the trimming region.
	 */
	virtual bool insideTrimmingRegion(const rw::math::Vector3D<>& P) const { return true; }

	/**
	 * @brief Get the normal of the surface at a specific point, \b x, lying on the surface.
	 *
	 * For the point on the implicit surface, where \f$ F(\mathbf{x})=0 \f$ ,
	 * the normal is the direction of the gradient \f$\frac{\nabla \mathbf{F}}{\|\nabla \mathbf{F}\|}\f$ .
	 *
	 * @param x [in] a point on the surface.
	 * @return the normal to the surface at \b x .
	 * @see the gradient function to find the gradient.
	 */
	virtual rw::math::Vector3D<> normal(const rw::math::Vector3D<>& x) const { return normalize(gradient(x)); }

	/**
	 * @brief Get the gradient, \f$\nabla \mathbf{F}\f$, of the surface at a specific point, \b x, lying on the surface.
	 *
	 * The gradient is the vector of partial derivatives
	 * \f$ \nabla \mathbf{F} = \begin{bmatrix}\frac{\partial F}{\partial x} & \frac{\partial F}{\partial y} & \frac{\partial F}{\partial z} \end{bmatrix}^T \f$
	 *
	 * @param x [in] a point on the surface.
	 * @return the gradient, \f$\nabla \mathbf{F}\f$, of the surface at \b x .
	 * @see the normal function to find the normal to the surface (the normalized gradient).
	 */
	virtual rw::math::Vector3D<> gradient(const rw::math::Vector3D<>& x) const = 0;

private:
	inline virtual Surface::Ptr doTransformSurface(const rw::math::Transform3D<>& T) const { return doTransformImplicitSurface(T); }
	inline virtual Surface::Ptr doTransformSurface(const rw::math::Vector3D<>& P) const { return doTransformImplicitSurface(P); }
	inline virtual Surface::Ptr doScaleSurface(double factor) const { return doScaleImplicitSurface(factor); }
	inline virtual Surface::Ptr doCloneSurface() const { return doCloneImplicitSurface(); }

	virtual ImplicitSurface::Ptr doTransformImplicitSurface(const rw::math::Transform3D<>& T) const = 0;
	virtual ImplicitSurface::Ptr doTransformImplicitSurface(const rw::math::Vector3D<>& P) const = 0;
	virtual ImplicitSurface::Ptr doScaleImplicitSurface(double factor) const = 0;
	virtual ImplicitSurface::Ptr doCloneImplicitSurface() const = 0;
};
//! @}
} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_ANALYTIC_IMPLICITSURFACE_HPP_ */
