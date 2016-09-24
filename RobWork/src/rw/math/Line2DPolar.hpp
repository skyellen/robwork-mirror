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

#ifndef RW_MATH_LINE2DPOLAR_HPP
#define RW_MATH_LINE2DPOLAR_HPP

#include "Pose2D.hpp"
#include "Line2D.hpp"
#include "Vector2D.hpp"

#include <rw/common/Serializable.hpp>

namespace rw {
namespace math {

/**
 * @brief Describes a line in 2D in polar coordinates.
 */
class Line2DPolar
{
public:
	/**
	 * @brief constructor
	 *
	 * rho * (cos(theta), sin(theta)) is the point on the line nearest to origo.
	 *
	 * @param rho [in] distance to the point on line which is closest to origo
	 * @param theta [in] angle from x-axis up to the line that connects the origo and the
	 * point on the line that is closest to origo.
	 */
	Line2DPolar(double rho = 0, double theta = 0);

	/**
	 * @brief constructor
	 *
	 * @param pnt [in] is any point on the line
	 * @param theta [in]  angle in radians from x-axis up to the line that connects the origo and the
	 * point on the line that is closest to origo.
	 */
	Line2DPolar(const Vector2D<>& pnt, double theta);

	/**
	 * @brief constructor - The line moving through the segment from 'start' to 'end'.
	 * @param start [in] point on line
	 * @param end [in] point on line
	 */
	Line2DPolar(const Vector2D<>& start, const Vector2D<>& end);

	/**
	 * @brief constructor - The line moving through the line segment.
	 * @param line [in] the line described as a segment
	 */
	Line2DPolar(const Line2D& line);

	/**
	 * @brief the shortest distance from origo the line
	 * @return
	 */
	double getRho() const { return _rho; }

	/**
	 * @brief angle in radians from x-axis up to the line that connects the origo and the
	 * point on the line that is closest to origo.
	 * @return
	 */
	double getTheta() const { return _theta; }

	//! @brief get normal of line
	Vector2D<> calcNormal() const;

	//! @brief The L_2 distance from 'pnt' to the line.
	double dist2(const Vector2D<>& pnt);


    /**
	 * @brief
     */
	/*
	friend const Line2DPolar operator*(const Transform2D<>& aTb, const Line2DPolar& bP){
    	return aTb._R * bP + aTb._d ;
    }

    friend const Line2DPolar operator*(const Pose2D<>& aTb, const Line2DPolar& bP){

    	return aTb._R * bP + aTb._d ;
    }
	*/

	//! The point for the projection of 'pnt' onto 'line'.
	static
	Vector2D<> projectionPoint(const Line2DPolar& line, const Vector2D<>& pnt);

	//! A supporting point on the line (equal to rho * normal).
	static Vector2D<> linePoint(const Line2DPolar& line);

	/**
	 * @brief The vector for the projection of \b pnt onto the normal of \b line.
	 * @param line [in] a line.
	 * @param pnt [in] a point.
	 * @return the projection vector.
	 */
	static Vector2D<> normalProjectionVector(const Line2DPolar& line, const Vector2D<>& pnt);

	// Print the line to stdout.
	//static void print(const LinePolar& line);

	/**
	 * @brief \b line given relative to the coordinate frame of \b pose.
	 * @param pose [in] the pose.
	 * @param line [in] the line.
	 * @return a Line2DPolar.
	 */
	static Line2DPolar lineToLocal(
		const Pose2D<>& pose,
		const Line2DPolar& line);

private:
	double _rho;
	double _theta;
};

}
}


namespace rw{ namespace common {
    class OutputArchive; class InputArchive;
namespace serialization {
	/**
	 * @copydoc rw::common::serialization::write
	 * @relatedalso rw::math::Line2DPolar
	 */
	template<> void write(const rw::math::Line2DPolar& sobject, rw::common::OutputArchive& oarchive, const std::string& id);

	/**
	 * @copydoc rw::common::serialization::read
	 * @relatedalso rw::math::Line2DPolar
	 */
	template<> void read(rw::math::Line2DPolar& sobject, rw::common::InputArchive& iarchive, const std::string& id);
}}} // end namespaces

#endif
