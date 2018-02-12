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

#ifndef RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICUTIL_HPP_
#define RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICUTIL_HPP_

/**
 * @file QuadraticUtil.hpp
 *
 * \copydoc rw::geometry::QuadraticUtil
 */

#include <rw/geometry/OBB.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw {
namespace geometry {
class QuadraticSurface;
class QuadraticCurve;

//! @addtogroup geometry

//! @{
/**
 * @brief Utility functions for operations on quadratic surfaces and curves.
 *
 * Functions are provided to find approximate closest points between different combinations of quadratics surfaces and curves.
 */
class QuadraticUtil {
public:
	//! @brief Type for a pair of points.
	typedef std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > PointPair;

	/**
	 * @brief Find and approximation of the closest points between two surfaces \b s1 and \b s2.
	 *
	 * The approximation is valid when the distance is small.
	 * Only closest points within the trimming regions will give a result.
	 *
	 * @param s1 [in] the first surface.
	 * @param s2 [in] the second surface.
	 * @param result [out] the result (only valid if the function returns true).
	 * @param epsIn [in] threshold for penetration depth.
	 * @param epsOut [in] threshold for distance.
	 * @return true if there is a closest point pair within the given limits.
	 */
	static bool closestPointsApproximation(const QuadraticSurface& s1, const QuadraticSurface& s2, PointPair& result, double epsIn, double epsOut);

	/**
	 * @brief Find and approximation of the closest points between a surface \b s and a curve \b c.
	 *
	 * The approximation is valid when the distance is small.
	 * Only closest points within the trimming region of surface \b s and within the limits of curve \b c will give a result.
	 *
	 * @param s [in] the surface.
	 * @param c [in] the curve.
	 * @param result [out] the result (only valid if the function returns true).
	 * @param epsIn [in] threshold for penetration depth.
	 * @param epsOut [in] threshold for distance.
	 * @return true if there is a closest point pair within the given limits.
	 */
	static bool closestPointsApproximation(const QuadraticSurface& s, const QuadraticCurve& c, PointPair& result, double epsIn, double epsOut);

	/**
	 * @brief Find an approximation of the curve parameter values of curves \b ca and \b cb at the closest points.
	 *
	 * The approximation is valid when the distance is small.
	 * Only closest points within the limits of both curves will give a result.
	 *
	 * @param sa1 [in] one of the surfaces the first curve \b ca is lying on.
	 * @param sa2 [in] the other surface the first curve \b ca is lying on.
	 * @param ca [in] the first curve.
	 * @param cb [in] the second curve.
	 * @return a list of pairs of curve parameter values at the closest points.
	 */
	static std::vector<std::pair<double,double> > closestTimesApproximation(const QuadraticSurface& sa1, const QuadraticSurface& sa2, const QuadraticCurve& ca, const QuadraticCurve& cb);

private:
	/**
	 * @brief Find an approximation of the closest points between curves \b ca and \b cb.
	 *
	 * The approximation is valid when the distance is small.
	 * Only closest points within the limits of curve \b cb will give a result.
	 *
	 * @param sa1 [in] one of the surfaces the first curve \b ca is lying on.
	 * @param sa2 [in] the other surface the first curve \b ca is lying on.
	 * @param ca [in] the first curve.
	 * @param cb [in] the second curve.
	 * @param epsIn [in] threshold for penetration depth.
	 * @param epsOut [in] threshold for distance.
	 * @return a list of the closest point pairs within the given limits.
	 */
	static std::vector<PointPair> closestPointsApproximation(const QuadraticSurface& sa1, const QuadraticSurface& sa2, const QuadraticCurve& ca, const QuadraticCurve& cb, double epsIn, double epsOut);

	/**
	 * @brief Find an approximation of the closest points between the curve on surfaces \b sa and \b sb and the curve \b c.
	 *
	 * The approximation is valid when the distance is small.
	 * Only closest points within the limits of curve \b c will give a result.
	 * The curve between surfaces \b sa and \b sb, is not checked with curve limits or trimming region conditions.
	 *
	 * @param sa [in] one of the surfaces the first curve is lying on.
	 * @param sb [in] the other surface the first curve is lying on.
	 * @param c [in] the second curve.
	 * @param epsIn [in] threshold for penetration depth.
	 * @param epsOut [in] threshold for distance.
	 * @return a list of the closest point pairs within the given limits.
	 */
	static std::vector<PointPair> closestPointsApproximation(const QuadraticSurface& sa, const QuadraticSurface& sb, const QuadraticCurve& c, double epsIn, double epsOut);

	/**
	 * @brief Find an approximation of the curve parameter when the curve \b c is at its closest points to the curve between surfaces \b sa and \b sb.
	 *
	 * The approximation is valid when the distance is small.
	 * No limits or trimming conditions is checked.
	 *
	 * @param sa [in] one of the surfaces the first curve is lying on.
	 * @param sb [in] the other surface the first curve is lying on.
	 * @param c [in] the second curve.
	 * @return a vector of one or more curve parameter values where there is a local minimum between of the distance between the curves.
	 */
	static std::vector<double> closestTimesApproximation(const QuadraticSurface& sa, const QuadraticSurface& sb, const QuadraticCurve& c);

	QuadraticUtil() {}
	virtual ~QuadraticUtil() {}

	static Eigen::Matrix3d adjugate(const Eigen::Matrix3d& A);
};
//! @}
} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICUTIL_HPP_ */
