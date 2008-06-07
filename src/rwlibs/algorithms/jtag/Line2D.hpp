#ifndef RWLIBS_ALGORITHMS_LINE2D_HPP_
#define RWLIBS_ALGORITHMS_LINE2D_HPP_

#include <rw/math/Vector2D.hpp>

namespace rwlibs {
namespace algorithms {


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
	
	double calcAngle(const Line2D &line);
	
	double calcAngle();
	
	/**
	 * @brief calculates the shortest distance between point v and the infinite
	 * line.
	 * @param 
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
	rw::math::Vector2D<>& p1(){
		return _p1;
	};	

	/**
	 * @brief second point on the line
	 */
	rw::math::Vector2D<>& p2(){
		return _p2;
	};
	
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

#endif /*LINE2D_HPP_*/
