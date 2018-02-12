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

#ifndef RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICCURVE_HPP_
#define RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICCURVE_HPP_

/**
 * @file QuadraticCurve.hpp
 *
 * \copydoc rw::geometry::QuadraticCurve
 */

#include <rw/geometry/analytic/ParametricCurve.hpp>
#include <rw/geometry/OBB.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw {
namespace geometry {
//! @addtogroup geometry

//! @{
/**
 * @brief A quadratic curve.
 *
 * A quadratic curve is given explicitly by the expression \f$ \mathbf{p} = \mathbf{c} + r(t) \mathbf{u} + s(t) \mathbf{v} \f$ where \f$ \mathbf{c},\mathbf{u},\mathbf{v},\mathbf{p} \in \mathbb{R}^3 \f$ and \f$ \mathbf{u}^T \mathbf{v} = 0\f$ .
 *
 * The following four types of curves are possible:
 * - Ellipse: \f$ (r,s)=(\sin t,\cos t) \f$
 * - Hyperbola: \f$ (r,s)=(\sinh t,\cosh t) \f$
 * - Line: \f$ (r,s)=(t,0) \f$
 * - Parabola: \f$ (r,s)=(t,t^2) \f$
 */
class QuadraticCurve: public ParametricCurve {
public:
	//! @brief Smart pointer type for QuadraticCurve.
	typedef rw::common::Ptr<QuadraticCurve> Ptr;

	//! @brief Smart pointer type for a const QuadraticCurve.
	typedef rw::common::Ptr<const QuadraticCurve> CPtr;

	//! @brief The four possible curve types.
	typedef enum Type {
		Elliptic, //!< Ellipse \f$ \mathbf{p} = \mathbf{c} + \mathbf{u} \sin t + \mathbf{v} \cos t\f$
		Hyperbola,//!< Hyperbola \f$ \mathbf{p} = \mathbf{c} + \mathbf{u} \sinh t + \mathbf{v} \cosh t\f$
		Line,     //!< Line \f$ \mathbf{p} = \mathbf{c} + \mathbf{u} t\f$
		Parabola  //!< Parabola \f$ \mathbf{p} = \mathbf{c} + \mathbf{u} t + \mathbf{v} t^2\f$
	} Type;

	/**
	 * @brief Construct quadratic curve given by the expression \f$ \mathbf{p} = \mathbf{c} + r(t) \mathbf{u} + s(t) \mathbf{v} \f$, where u and v are orthogonal vectors.
	 * @param c [in] offset of curve.
	 * @param u [in] first direction.
	 * @param v [in] second direction.
	 * @param type [in] the type of curve.
	 */
	QuadraticCurve(const rw::math::Vector3D<>& c, const rw::math::Vector3D<>& u, const rw::math::Vector3D<>& v, const Type& type);

	//! @brief Destrcutor.
	virtual ~QuadraticCurve();

	// Curve interface
	//! @copydoc ParametricCurve::transform(const rw::math::Transform3D<>&) const
	QuadraticCurve::Ptr transform(const rw::math::Transform3D<>& T) const;

	//! @copydoc ParametricCurve::transform(const rw::math::Vector3D<>&) const
	QuadraticCurve::Ptr transform(const rw::math::Vector3D<>& P) const;

	//! @copydoc ParametricCurve::scale
	QuadraticCurve::Ptr scale(double factor) const;

	//! @copydoc ParametricCurve::reverse
	QuadraticCurve::Ptr reverse() const;

	//! @copydoc ParametricCurve::clone
	QuadraticCurve::Ptr clone() const;

	//! @copydoc ParametricCurve::extremums
	virtual std::pair<double,double> extremums(const rw::math::Vector3D<>& dir) const;

	//! @copydoc ParametricCurve::discretizeAdaptive
	virtual std::list<rw::math::Vector3D<> > discretizeAdaptive(double stepsPerRevolution) const;

	//! @copydoc ParametricCurve::obr
	virtual OBB<> obr() const;

	//! @copydoc ParametricCurve::closestPoints
	virtual std::vector<rw::math::Vector3D<> > closestPoints(const rw::math::Vector3D<>& p) const;

	// ParametricCurve interface
	//! @copydoc ParametricCurve::x
	virtual rw::math::Vector3D<> x(double t) const;

	//! @copydoc ParametricCurve::dx
	virtual rw::math::Vector3D<> dx(double t) const;

	//! @copydoc ParametricCurve::ddx
	virtual rw::math::Vector3D<> ddx(double t) const;

	//! @copydoc ParametricCurve::x(double) const
	virtual rw::math::Vector3D<> operator()(double t) const;

	//! @copydoc ParametricCurve::hasLimits
	virtual bool hasLimits() const { return _hasLimits; }

	//! @copydoc ParametricCurve::limits
	virtual const std::pair<double,double>& limits() const { return _limits; }

	//! @copydoc ParametricCurve::inLimits
	virtual bool inLimits(double t) const;

	//! @copydoc ParametricCurve::setLimits
	virtual void setLimits(const std::pair<double,double>& limits);

	//! @copydoc ParametricCurve::curvature
	virtual double curvature(double t) const;

	//! @copydoc ParametricCurve::closestTimes
	virtual std::vector<double> closestTimes(const rw::math::Vector3D<>& p) const;

	//! @copydoc ParametricCurve::closestTime
	virtual double closestTime(const rw::math::Vector3D<>& p) const;

	// QuadraticCurve
	/**
	 * @brief The point c.
	 * @return the point \f$ c \in \mathbb{R}^3 \f$.
	 */
	const rw::math::Vector3D<>& c() const { return _c; }

	/**
	 * @brief The vector u.
	 * @return the vector \f$ u \in \mathbb{R}^3 \f$.
	 */
	const rw::math::Vector3D<>& u() const { return _u; }

	/**
	 * @brief The vector v.
	 * @return the vector \f$ v \in \mathbb{R}^3 \f$.
	 */
	const rw::math::Vector3D<>& v() const { return _v; }

	/**
	 * @brief Get the type of curve.
	 * @return the type of curve.
	 */
	Type type() const { return _type; }

private:
	double r(double t) const;
	double s(double t) const;

	std::list<double> discretizeEllipse(double stepsPerRevolution) const;

	virtual ParametricCurve::Ptr doScaleParametricCurve(double factor) const { return scale(factor); }
	virtual ParametricCurve::Ptr doTransformParametricCurve(const rw::math::Vector3D<>& P) const { return transform(P); }
	virtual ParametricCurve::Ptr doTransformParametricCurve(const rw::math::Transform3D<>& T) const { return transform(T); }
	virtual ParametricCurve::Ptr doReverseParametricCurve() const { return reverse(); }
	virtual ParametricCurve::Ptr doCloneParametricCurve() const { return clone(); }

	const rw::math::Vector3D<> _c;
	const rw::math::Vector3D<> _u;
	const rw::math::Vector3D<> _v;
	const Type _type;

	bool _hasLimits;
	std::pair<double,double> _limits;
};
//! @}
} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICCURVE_HPP_ */
