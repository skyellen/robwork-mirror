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

#ifndef RWSIM_UTIL_LINEPOLAR_HPP
#define RWSIM_UTIL_LINEPOLAR_HPP

#include "Pose2D.hpp"
#include "P2D.hpp"
#include <boost/numeric/ublas/vector.hpp>


namespace rwsim {
namespace util {

	class LinePolar
	{
	public:
		// rho * (cos(theta), sin(theta)) is the point on the line nearest to origo.
		LinePolar(double rho = 0, double theta = 0);

		// 'pnt' is any point on the line, and theta is the usual angle.
		static LinePolar make(const P2D& pnt, double theta);

		// The line moving through the segment from 'start' to 'end'.
		static LinePolar make(const P2D& start, const P2D& end);

		double getRho() const { return _rho; }
		double getTheta() const { return _theta; }
		const P2D& getNormal() const { return _normal; }

		// The L_2 distance from 'pnt' to the line.
		double dist2(const P2D& pnt);

		// The point for the projection of 'pnt' onto 'line'.
		static
		P2D projectionPoint(const LinePolar& line, const P2D& pnt);

		// A supporting point on the line (equal to rho * normal).
		static
		P2D linePoint(const LinePolar& line);

		// The vector for the projection of 'pnt' onto the normal of 'line'.
		static
		P2D normalProjectionVector(const LinePolar& line, const P2D& pnt);

		// Print the line to stdout.
		static
		void print(const LinePolar& line);

		// An ublas vector of (rho, theta).
		static
		boost::numeric::ublas::vector<double> toUblas(const LinePolar& line);

		// 'line' given relative to the coordinate frame of 'pose'.
		static
		LinePolar lineToLocal(
			const Pose2D& pose,
			const LinePolar& line);

	public:
		double _rho;
		double _theta;
		P2D _normal;
	};
}
}

#endif
