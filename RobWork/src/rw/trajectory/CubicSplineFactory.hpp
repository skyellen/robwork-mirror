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


#ifndef RW_TRAJECTORY_CUBICSPLINEFACTORY_HPP
#define RW_TRAJECTORY_CUBICSPLINEFACTORY_HPP

/**
 * @file CubicSplineFactory.hpp
 */


#include "InterpolatorTrajectory.hpp"
#include "Path.hpp"

namespace rw {
namespace trajectory {

/** @addtogroup trajectory */
/*@{*/

/**
 * @brief Factory for creating cubic splines
 *
 *
 */
class CubicSplineFactory
{
public:

    /**
     * @brief constructs a free/natural cubic spline
     * A natural cubic spline has free boundary conditions. Only one condition
     * can be said for the end points namely acceleration is zero.
     * * The spline passes through each data point.
     * * The spline forms a continuous function over [a,b].
     * * The spline forms a smooth function.
     * * The second derivative is continuous.
     *
     * @param qpath [in] a list of points that the spline should intersect
     * @param timeStep [in] the duration of each spline path
     */
	static InterpolatorTrajectory<rw::math::Q>::Ptr makeNaturalSpline(QPath::Ptr qpath, double timeStep=1.0);

    /**
     * @brief constructs a natural cubic spline, see above.
     *
     * @param tqpath [in] a list of points with associated timestaps. The spline will intersect
     * the points at the time specified in \b tqpath
     * @param offset [in]
	 * @return a trajectory of CubicSplineInterpolators
     */
	static InterpolatorTrajectory<rw::math::Q>::Ptr makeNaturalSpline(TimedQPath::Ptr tqpath);

	/**
	 * @brief Construct a natural cubic spline. See documentation of CubicSplineFactory::makeNaturalSpline(QPath::Ptr, double)	 
	 *
	 * @param qpath [in] Path to follow
	 * @param times [in] Times associated to the different configurations in \b qpath	 
	 * @return a trajectory of CubicSplineInterpolators
	 */
	static InterpolatorTrajectory<rw::math::Q>::Ptr makeNaturalSpline(const QPath& qpath, const std::vector<double>& times);


	/**
	 * @brief creates a clamped spline trajectory with equally spaced
	 * via points. That is time between samples is 1. A clamped spline controls
	 * the velocity in the end points. The acceleration is 0 in the end points.
	 * @param qpath [in] the path over which the spline should be generated.
	 * @param dqStart [in] the velocity in the first point
	 * @param dqEnd [in] the velocity in the last point.
	 * @return a trajectory of CubicSplineInterpolators
	 */
	static InterpolatorTrajectory<rw::math::Q>::Ptr makeClampedSpline(QPath::Ptr qpath,
			const rw::math::Q& dqStart,
		    const rw::math::Q& dqEnd,
		    double timeStep = 1.0);

	/**
	 * @brief creates a clamped spline trajectory where the timed label is used
	 * to determine the time between samples. A clamped spline controls
	 * the velocity in the end points. The acceleration is 0 in the end points.
	 * @param qpath [in] the path over which the spline should be generated.
	 * @param dqStart [in] the velocity in the first point
	 * @param dqEnd [in] the velocity in the last point.
	 * @return a trajectory of CubicSplineInterpolators
	 */
	static InterpolatorTrajectory<rw::math::Q>::Ptr makeClampedSpline(TimedQPath::Ptr tqpath,
			const rw::math::Q& dqStart,
		    const rw::math::Q& dqEnd);


	/**
	 * @brief creates a clamped spline trajectory where the timed label is used
	 * to determine the time between samples. A clamped spline controls
	 * the velocity in the end points. The acceleration is 0 in the end points.
	 * @param qpath [in] the path over which the spline should be generated.
	 * @param times [in] the times associated to the configurations in \b qpath.
	 * @param dqStart [in] the velocity in the first point
	 * @param dqEnd [in] the velocity in the last point.
	 * @return a trajectory of CubicSplineInterpolators
	 */
	static InterpolatorTrajectory<rw::math::Q>::Ptr makeClampedSpline(const QPath& qpath, const std::vector<double>& times, const rw::math::Q& dqStart, const rw::math::Q& dqEnd);


#ifdef RW_USE_UBLAS_LAPACK
	/**
	 * @brief LAPACK based version of makeNaturalSpline
	 */
	static InterpolatorTrajectory<rw::math::Q>::Ptr makeNaturalSplineLapack(QPath::Ptr qpath, double timeStep=1.0);

	/**
	 * @brief LAPACK based version of makeNaturalSpline
	 */
	static InterpolatorTrajectory<rw::math::Q>::Ptr makeNaturalSplineLapack(TimedQPath::Ptr tqpath);

	/**
	 * @brief LAPACK based version of makeNaturalSpline
	 */
	static InterpolatorTrajectory<rw::math::Q>::Ptr makeClampedSplineLapack(QPath::Ptr qpath, const rw::math::Q& dqStart, const rw::math::Q& dqEnd,	double timeStep=1.0);

	/**
	 * @brief LAPACK based version of makeNaturalSpline
	 */
	static InterpolatorTrajectory<rw::math::Q>::Ptr makeClampedSplineLapack(TimedQPath::Ptr tqpath, const rw::math::Q& dqStart, const rw::math::Q& dqEnd);

#endif

private:
	CubicSplineFactory();

	virtual ~CubicSplineFactory();
};


/** @} */

} //end namespace trajectory
} //end namespace rw

#endif /*RW_TRAJECTORY_CUBICSPLINEFACTORY_HPP_*/
