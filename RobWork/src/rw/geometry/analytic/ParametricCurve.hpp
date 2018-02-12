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

#ifndef RW_GEOMETRY_ANALYTIC_PARAMETRICCURVE_HPP_
#define RW_GEOMETRY_ANALYTIC_PARAMETRICCURVE_HPP_

/**
 * @file ParametricCurve.hpp
 *
 * \copydoc rw::geometry::ParametricCurve
 */

#include "Curve.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/geometry/OBB.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw {
namespace geometry {
//! @addtogroup geometry

//! @{
/**
 * @brief Interface for parametric curves. A parametric curve, \f$ \mathbf{p}(t) \in \mathbb{R}^3 \f$, has its points given as a function of a single parameter, \f$ t \in \mathbb{R}\f$.
 *
 * Parametric curves have functions for evaluation of points, derivatives and curvature.
 * A parmateric curve can be limited, and it is possible to find closest points with a given point.
 */
class ParametricCurve: public Curve {
public:
	//! @brief Smart pointer type for ParametricCurve.
	typedef rw::common::Ptr<ParametricCurve> Ptr;

	//! @brief Smart pointer type for a const ParametricCurve.
	typedef rw::common::Ptr<const ParametricCurve> CPtr;

	//! @brief Constructor.
	ParametricCurve() {}

	//! @brief Destructor.
	virtual ~ParametricCurve() {}

	//! @copydoc Curve::transform(const rw::math::Transform3D<>&) const
	inline ParametricCurve::Ptr transform(const rw::math::Transform3D<>& T) const { return doTransformParametricCurve(T); }

	//! @copydoc Curve::transform(const rw::math::Vector3D<>&) const
	inline ParametricCurve::Ptr transform(const rw::math::Vector3D<>& P) const { return doTransformParametricCurve(P); }

	//! @copydoc Curve::scale
	inline ParametricCurve::Ptr scale(double factor) const { return doScaleParametricCurve(factor); }

	//! @copydoc Curve::reverse
	inline ParametricCurve::Ptr reverse() const { return doReverseParametricCurve(); }

	//! @copydoc Curve::clone
	inline ParametricCurve::Ptr clone() const { return doCloneParametricCurve(); }

	//! @copydoc Curve::extremums
	virtual std::pair<double,double> extremums(const rw::math::Vector3D<>& dir) const = 0;

	//! @copydoc Curve::discretizeAdaptive
	virtual std::list<rw::math::Vector3D<> > discretizeAdaptive(double stepsPerRevolution) const = 0;

	//! @copydoc Curve::obr
	virtual OBB<> obr() const = 0;

	//! @copydoc Curve::closestPoints
	virtual std::vector<rw::math::Vector3D<> > closestPoints(const rw::math::Vector3D<>& p) const = 0;

	/**
	 * @brief Evaluate a point on the curve.
	 * @param t [in] the parameter to find point for.
	 * @return the vector \f$ p \in \mathbb{R}^3 \f$ .
	 */
	virtual rw::math::Vector3D<> x(double t) const = 0;

	/**
	 * @brief Evaluate the derivative in a point on the curve.
	 * @param t [in] the parameter to find derivative for.
	 * @return a derivative vector \f$ p \in \mathbb{R}^3 \f$ .
	 */
	virtual rw::math::Vector3D<> dx(double t) const = 0;

	/**
	 * @brief Evaluate the second derivative in a point on the curve.
	 * @param t [in] the parameter to find second derivative for.
	 * @return a second derivative vector \f$ p \in \mathbb{R}^3 \f$ .
	 */
	virtual rw::math::Vector3D<> ddx(double t) const = 0;

	//! @copydoc x(double) const
	virtual rw::math::Vector3D<> operator()(double t) const = 0;

	/**
	 * @brief Check if the curve is limited.
	 * @return true if curve is limited, false otherwise.
	 */
	virtual bool hasLimits() const = 0;

	/**
	 * @brief Get the limits of the curve segment.
	 *
	 * The returned values are only valid when hasLimits() returns true.
	 *
	 * @return the minimum and maximum parameter values on the curve.
	 */
	virtual const std::pair<double,double>& limits() const = 0;

	/**
	 * @brief Check if the parameter \b t is inside the limits set for the curve.
	 * @param t [in] the parameter to check.
	 * @return true if inside limits, false otherwise.
	 */
	virtual bool inLimits(double t) const = 0;

	/**
	 * @brief Set parameter limits for the curve.
	 * @param limits [in] the minimum and maximum parameter values on the curve.
	 */
	virtual void setLimits(const std::pair<double,double>& limits) = 0;

	/**
	 * @brief The curvature in a given point on the curve.
	 *
	 * This function does not take the limits into account.
	 *
	 * @param t [in] the parameter to evaluate the curvature for.
	 * @return the curvature.
	 */
	virtual double curvature(double t) const = 0;

	/**
	 * @brief Get the parameter values where the curve is closest to a point \b p.
	 *
	 * Notice that the limits are taken into account.
	 *
	 * @param p [in] the point to find closest values for.
	 * @return a list of parameter values.
	 */
	virtual std::vector<double> closestTimes(const rw::math::Vector3D<>& p) const = 0;

	/**
	 * @brief Get the parameter value where the curve is closest to a point \b p.
	 *
	 * Notice that the limits are taken into account.
	 *
	 * @param p [in] the point to find closest values for.
	 * @return the point on the curve closest to \b p. If multiple points are equally close to \b p, only one of those points are returned.
	 */
	virtual double closestTime(const rw::math::Vector3D<>& p) const = 0;

private:
	virtual Curve::Ptr doScaleCurve(double factor) const { return doScaleParametricCurve(factor); }
	virtual Curve::Ptr doTransformCurve(const rw::math::Vector3D<>& P) const { return doTransformParametricCurve(P); }
	virtual Curve::Ptr doTransformCurve(const rw::math::Transform3D<>& T) const { return doTransformParametricCurve(T); }
	virtual Curve::Ptr doReverseCurve() const { return doReverseParametricCurve(); }
	virtual Curve::Ptr doCloneCurve() const { return doCloneParametricCurve(); }

	virtual ParametricCurve::Ptr doScaleParametricCurve(double factor) const = 0;
	virtual ParametricCurve::Ptr doTransformParametricCurve(const rw::math::Vector3D<>& P) const = 0;
	virtual ParametricCurve::Ptr doTransformParametricCurve(const rw::math::Transform3D<>& T) const = 0;
	virtual ParametricCurve::Ptr doReverseParametricCurve() const = 0;
	virtual ParametricCurve::Ptr doCloneParametricCurve() const = 0;
};
//! @}
} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_ANALYTIC_PARAMETRICCURVE_HPP_ */
