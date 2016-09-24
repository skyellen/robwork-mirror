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


#ifndef RW_MATH_LINE2D_HPP
#define RW_MATH_LINE2D_HPP

#include "Vector2D.hpp"

#include <rw/common/Serializable.hpp>

namespace rw {
namespace math {


/**
 * @brief Describes a line segment in 2D.
 *
 */
class Line2D
{
public:

	/**
	 * @brief definition of intersection result values for the intersection test
	 * between two lines.
	 */
	enum IntersectResult {
		PARALLEL, //! Two lines are parallel
		COINCIDENT, //! Two lines are parallel and coinciding
		INTERSECTS //! Two lines intersects at one point
		};

public:
	/**
	 * @brief Constructor
	 */
	Line2D(){};

	/**
	 * @brief Creates a line between that intersect the two points p1 and p2.
	 * @param p1 [in] point
	 * @param p2 [in] point
	 */
	Line2D(const rw::math::Vector2D<>& p1, const rw::math::Vector2D<>& p2);

	/**
	 * @brief Creates a line between that intersect the two points (x1,y1) and (x2,y2).
	 * @param x1 [in] x coordinate of point 1
	 * @param y1 [in] y coordinate of point 1
	 * @param x2 [in] x coordinate of point 2
	 * @param y2 [in] y coordinate of point 2
	 */
	Line2D(double x1, double y1, double x2, double y2);

	/**
	 * @brief Destructor
	 */
	virtual ~Line2D();

	/**
	 * @brief calculates the intersection between two lines. A intersection
	 * point is only saved in res if the two lines are not parallel or coincident.
	 * The intersection test does not take the segments into acount.
	 * @param line [in] the line two test against
	 * @param res [out] the point of intersection
	 * @return the intersection type
	 */
	IntersectResult getIntersect(Line2D &line, rw::math::Vector2D<> &res);

	/**
	 * @brief calculates the angle between this line and \b line
	 * @param line [in] a line
	 * @return the angle from this line to \b line
	 */
	double calcAngle(const Line2D &line);

    /**
     * @brief calculates the angle between the projection of this line onto
     * yz-plane and the x-axis
     * @return the angle
     */
	double calcAngle();

	/**
	 * @brief calculates the shortest distance between point v and the infinite
	 * line.
	 * @param v [in] Point to which to calculate distance
	 */
	double calcDist(const rw::math::Vector2D<>& v) const;

	/**
	 * @brief gets the length of thi line segment.
	 * @return line segment length
	 */
	double getLength(){
		rw::math::Vector2D<> diff = _p1-_p2;
		return diff.norm2();
	}

	/**
	 * @brief first point on the line
	 */
	rw::math::Vector2D<>& p1(){return _p1;}

	//! @copydoc p1()
	const rw::math::Vector2D<>& p1() const {return _p1;}

	/**
	 * @brief second point on the line
	 */
	rw::math::Vector2D<>& p2(){ return _p2; }

	//! @copydoc p2()
	const rw::math::Vector2D<>& p2() const {return _p2;}

	/**
	 * @brief calculates the unit normal of the line
	 */
    rw::math::Vector2D<> calcUnitNormal(){
        rw::math::Vector2D<> u = (_p2-_p1)/getLength();
        return rw::math::Vector2D<>(-u(1),u(0));
    }


private:
	rw::math::Vector2D<> _p1, _p2;
};

} // namespace algorithms
} // namespace rwlibs

namespace rw{ namespace common {
    class OutputArchive; class InputArchive;
namespace serialization {
	/**
	 * @copydoc rw::common::serialization::write
	 * @relatedalso rw::math::Line2D
	 */
	template<> void write(const rw::math::Line2D& sobject, rw::common::OutputArchive& oarchive, const std::string& id);

	/**
	 * @copydoc rw::common::serialization::read
	 * @relatedalso rw::math::Line2D
	 */
	template<> void read(rw::math::Line2D& sobject, rw::common::InputArchive& iarchive, const std::string& id);
}}} // end namespaces


#endif /*RW_MATH_LINE2D_HPP*/
