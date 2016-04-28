/*
 * BVCollider.hpp
 *
 *  Created on: 28-10-2008
 *      Author: jimali
 */

#ifndef RW_PROXIMITY_RSSDISTANCECALC_HPP_
#define RW_PROXIMITY_RSSDISTANCECALC_HPP_

#include "OBB.hpp"
#include "BVDistanceCalc.hpp"
#include <rw/math/Vector3D.hpp>
//#include "DistanceUtil.hpp"
#include <rw/math/Math.hpp>

namespace rw {
namespace proximity {

	/**
	 * @brief class for testing if two Oriented Bounding Boxes are overlapping
	 */

	template<class T=double>
	class RectDistanceCalc : public BVDistanceCalc<RectDistanceCalc<T>, rw::geometry::OBRectangle<T> > {
	public:
		typedef typename rw::geometry::OBB<> BVType;
		typedef T value_type;

		//! @brief constructor
		RectDistanceCalc(){};

		//! @brief destructor
		virtual ~RectDistanceCalc(){};

		/**
		 * @brief test if obbA intersects obbB. The aTb transform describe
		 * obbB relative to obbA's coordinate frame
		 */
		double distance(const rw::geometry::OBB<T>& obbA,
								 const rw::geometry::OBB<T>& obbB,
								 const rw::math::Transform3D<T>& aTb);


	    /**
	     * @brief calculates the distance between two segments, where one segment is defined in
	     * the x-y plane extending in either x or y direction (horizontal or vertical). the second segment
	     * is described in 3d with a start point \b p1, a direction \b d1 and a length \b d1_l.
	     *
	     * @param q1_x
	     * @param q1_y
	     * @param q1_l
	     * @param p1
	     * @param d1
	     * @param d1_l
	     * @return
	     */
	    void distanceSegments(
	                double &t,
	                double &u,
	                double a,
	                double b,
	                //const rw::math::Vector3D<>& Pb,
	                //const rw::math::Vector3D<>& B,
	                double ADotB,
	                double ADotT,
	                double BDotT)
	    {
	        using namespace rw::math;
	        // T is the direction vector from Pa to Pb
	        double denom = 1 - (ADotB*ADotB);
	        if (denom == 0){
	            t = 0;
	        } else {
	            t = Math::clamp( (ADotT - BDotT*ADotB)/denom, 0, a);
	        }

	        u = t*ADotB - BDotT;
	        if (u < 0){
	          u = 0;
	          t = Math::clamp( ADotT,0,a);
	        } else if (u > b){
	          u = b;
	          t = Math::clamp( u*ADotB + ADotT,0,a);
	        }
	    }
	};
/*
	template<class T>
	double RSSDistanceCalc<T>::distance(
		const rw::geometry::OBB<T>& obbA,
		const rw::geometry::OBB<T>& obbB,
		const rw::math::Transform3D<T>& aTb)
	{
	  using namespace rw::math;

	  T t, s;

	  const Vector3D<T> &a = obbA.getHalfLengths();
	  const Vector3D<T> &b = obbB.getHalfLengths();

	  // aTbf = fabs(aTb.R)
	  const Vector3D<T>& aPb = aTb.P();
	  const Rotation3D<T> &aRb = aTb.R();

	  const Rotation3D<T> &bRa = inverse(aRb);
	  const Vector3D<T>& bPa = -(bRa*aPb);

	  //std::cout << "aPb: " << aPb << std::endl;
	  //std::cout << "bPa: " << bPa << std::endl;

	  // the vectors describing the b halflength in A's reference frame
	  const Vector3D<T>& b_x = aRb*Vector3D<T>(b(0),0,0);
	  const Vector3D<T>& b_y = aRb*Vector3D<T>(0,b(1),0);

	  // the vectors describing the a halflength in B's reference frame
      const Vector3D<T>& a_x = bRa*Vector3D<T>(a(0),0,0);
      const Vector3D<T>& a_y = bRa*Vector3D<T>(0,a(1),0);

      const T AxDotBx = aRb(0,0);
      const T AxDotBy = aRb(0,1);
      const T AyDotBx = aRb(1,0);
      const T AyDotBy = aRb(1,1);

      // we define the edges of A and B as the segments
      // A_e1: {( a(0), a(1)), ( a(0),-a(1))}
      // A_e2: {(-a(0), a(1)), (-a(0),-a(1))}
      // A_e3: {( a(0), a(1)), (-a(0), a(1))}
      // A_e4: {( a(0),-a(1)), (-a(0),-a(1))}

	  // the closest features on A and B is found as the feature set where
	  // feature on B is in halfspace of feature A, and feature on A is in
	  // halfspace of feature on B.

	  // for each edge pair we do a closest feature test.

	  // test A_e1 against B_e1

      // is B_e1 inside halfspace of A_e1, and vice verse
      bool bInsideA = (a(0) < aPb(0)+b_x(0)+b_y(0)) && (a(0) < aPb(0)+b_x(0)-b_y(0));
      bool AInsideB = (b(0) < bPa(0)+a_x(0)+a_y(0)) && (b(0) < bPa(0)+a_x(0)-a_y(0));
	  if( bInsideA && AInsideB ){
	      //std::cout << "1";
	      // find closest points on segments
          //distanceSegments(t,u,2*a(1),2*b(1),AyDotBy,aPb(1)+b);
	      return DistanceUtil::distanceLineLine(
	              Vector3D<>(a(0),a(1),0), Vector3D<>(a(0),-a(1),0),
	              aPb+b_x+b_y, aPb+b_x-b_y );
	  }

      // is B_e1 inside halfspace of A_e2, and vice verse
      bInsideA = (-a(0) > aPb(0)+b_x(0)+b_y(0)) && (-a(0) > aPb(0)+b_x(0)-b_y(0));
      AInsideB = ( b(0) < bPa(0)-a_x(0)+a_y(0)) && ( b(0) < bPa(0)-a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){

          // find closest points on segments
          return DistanceUtil::distanceLineLine(
                  Vector3D<>(-a(0),a(1),0), Vector3D<>(-a(0),-a(1),0),
                  aPb+b_x+b_y, aPb+b_x-b_y );
      }

      // is B_e1 inside halfspace of A_e3, and vice verse
      bInsideA = (a(1) < aPb(1)+b_x(1)+b_y(1)) && (a(1) < aPb(1)+b_x(1)-b_y(1));
      AInsideB = (b(0) < bPa(0)+a_x(0)+a_y(0)) && (b(0) < bPa(0)+a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){

          return DistanceUtil::distanceLineLine(
                  Vector3D<>(a(0),a(1),0), Vector3D<>(-a(0),a(1),0),
                  aPb+b_x+b_y, aPb+b_x-b_y );
          // find closest points on segments
      }

      // is B_e1 inside halfspace of A_e4, and vice verse
      bInsideA = (-a(1) > aPb(1)+b_x(1)+b_y(1)) && (-a(1) > aPb(1)+b_x(1)-b_y(1));
      AInsideB = ( b(0) < bPa(0)+a_x(0)-a_y(0)) && ( b(0) < bPa(0)-a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){

          return DistanceUtil::distanceLineLine(
                  Vector3D<>(a(0),-a(1),0), Vector3D<>(-a(0),-a(1),0),
                  aPb+b_x+b_y, aPb+b_x-b_y );

          // find closest points on segments
      }




      // now B_e2, -x,+-y
      // is B_e2 inside halfspace of A_e1, and vice verse
      bInsideA = ( a(0) < aPb(0)-b_x(0)+b_y(0)) && ( a(0) < aPb(0)-b_x(0)-b_y(0));
      AInsideB = (-b(0) > bPa(0)+a_x(0)+a_y(0)) && (-b(0) > bPa(0)+a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){

          // find closest points on segments
          RW_THROW("5");
      }

      // is B_e2 inside halfspace of A_e2, and vice verse
      bInsideA = (-a(0) > aPb(0)-b_x(0)+b_y(0)) && (-a(0) > aPb(0)-b_x(0)-b_y(0));
      AInsideB = (-b(0) > bPa(0)-a_x(0)+a_y(0)) && (-b(0) > bPa(0)-a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){
          RW_THROW("6");

          //std::cout << "6";
          //std::cout << -b(0)<< ">" << bPa(0)<<"-" <<a_x(0)<<"+"<<a_y(0) << "&&" << -b(0) << ">" << bPa(0)-a_x(0)-a_y(0) << std::endl;
          //std::cout << Vector3D<>(-a(0),a(1),0)
          //          << Vector3D<>(-a(0),-a(1),0) << "\n"
          //          << aPb-b_x+b_y << aPb-b_x-b_y << "\n";

          return DistanceUtil::distanceLineLine(
                  Vector3D<>(-a(0),a(1),0), Vector3D<>(-a(0),-a(1),0),
                  aPb-b_x+b_y, aPb-b_x-b_y );
          // find closest points on segments
      }

      // is B_e2 inside halfspace of A_e3, and vice verse
      bInsideA = ( a(1) < aPb(1)-b_x(1)+b_y(1)) && ( a(1) < aPb(1)-b_x(1)-b_y(1));
      AInsideB = (-b(0) > bPa(0)+a_x(0)+a_y(0)) && (-b(0) > bPa(0)+a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){
          RW_THROW("7");
          // find closest points on segments
      }

      // is B_e2 inside halfspace of A_e4, and vice verse
      bInsideA = (-a(1) > aPb(1)-b_x(1)+b_y(1)) && (-a(1) > aPb(1)-b_x(1)-b_y(1));
      AInsideB = (-b(0) > bPa(0)+a_x(0)-a_y(0)) && (-b(0) > bPa(0)-a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){
          RW_THROW("8");
          // find closest points on segments
      }




      // now B_e3, +-x,+y
      // is B_e3 inside halfspace of A_e1, and vice verse
      bInsideA = (a(0) < aPb(0)+b_x(0)+b_y(0)) && (a(0) < aPb(0)-b_x(0)+b_y(0));
      AInsideB = (b(1) < bPa(1)+a_x(0)+a_y(1)) && (b(1) < bPa(1)+a_x(1)-a_y(1));
      if( bInsideA && AInsideB ){
          RW_THROW("9");
          // find closest points on segments
      }

      // is B_e3 inside halfspace of A_e2, and vice verse
      bInsideA = (-a(0) > aPb(0)+b_x(0)+b_y(0)) && (-a(0) > aPb(0)-b_x(0)+b_y(0));
      AInsideB = ( b(1) < bPa(1)-a_x(1)+a_y(1)) && ( b(1) < bPa(0)-a_x(1)-a_y(1));
      if( bInsideA && AInsideB ){
          RW_THROW("10");
          // find closest points on segments
      }

      // is B_e3 inside halfspace of A_e3, and vice verse
      bInsideA = (a(1) < aPb(1)+b_x(1)+b_y(1)) && (a(1) < aPb(1)-b_x(1)+b_y(1));
      AInsideB = (b(1) < bPa(1)+a_x(1)+a_y(1)) && (b(1) < bPa(1)+a_x(1)-a_y(1));
      if( bInsideA && AInsideB ){
          RW_THROW("11");
          // find closest points on segments
      }

      // is B_e1 inside halfspace of A_e4, and vice verse
      bInsideA = (-a(1) > aPb(1)+b_x(1)+b_y(1)) && (-a(1) > aPb(1)-b_x(1)+b_y(1));
      AInsideB = ( b(1) < bPa(1)+a_x(0)-a_y(1)) && ( b(1) < bPa(1)-a_x(1)-a_y(1));
      if( bInsideA && AInsideB ){
          RW_THROW("12");
          // find closest points on segments
          return DistanceUtil::distanceLineLine(
                  Vector3D<>(a(0),-a(1),0), Vector3D<>(-a(0),-a(1),0),
                  aPb+b_x+b_y, aPb+b_x-b_y );
      }




      // now B_e4, +-x,-y
      // is B_e4 inside halfspace of A_e1, and vice verse
      bInsideA = ( a(0) < aPb(0)+b_x(0)-b_y(0)) && ( a(0) < aPb(0)-b_x(0)-b_y(0));
      AInsideB = (-b(0) > bPa(0)+a_x(0)+a_y(0)) && (-b(0) > bPa(0)+a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){
          RW_THROW("13");
          // find closest points on segments
      }

      // is B_e4 inside halfspace of A_e2, and vice verse
      bInsideA = (-a(0) > aPb(0)+b_x(0)-b_y(0)) && (-a(0) > aPb(0)-b_x(0)-b_y(0));
      AInsideB = (-b(0) > bPa(0)-a_x(0)+a_y(0)) && (-b(0) > bPa(0)-a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){
          RW_THROW("14");
          // find closest points on segments
      }

      // is B_e4 inside halfspace of A_e3, and vice verse
      bInsideA = ( a(1) < aPb(1)+b_x(1)-b_y(1)) && ( a(1) < aPb(1)-b_x(1)-b_y(1));
      AInsideB = (-b(0) > bPa(0)+a_x(0)+a_y(0)) && (-b(0) > bPa(0)+a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){
          RW_THROW("15");
          // find closest points on segments
      }

      // is B_e4 inside halfspace of A_e4, and vice verse
      bInsideA = (-a(1) > aPb(1)+b_x(1)-b_y(1)) && (-a(1) > aPb(1)-b_x(1)-b_y(1));
      AInsideB = (-b(0) > bPa(0)+a_x(0)-a_y(0)) && (-b(0) > bPa(0)-a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){
          RW_THROW("16");
          // find closest points on segments
      }



      // we compute minimum distance between all possible segment pairs and return the minimum of these



	  // and we compute point rectangle plane distances

      RW_THROW("17");
	  return 0;  // should equal 0
	}
*/

	template<class T>
	void SegCoords(T& t, T& u,
	          const T& a, const T& b,
	          const T& A_dot_B,
	          const T& A_dot_T,
	          const T& B_dot_T)
	{
	  using namespace rw::math;
	  T denom = 1 - (A_dot_B)*(A_dot_B);

	  if (denom == 0) t = 0;
	  else
	  {
	    t = (A_dot_T - B_dot_T*A_dot_B)/denom;
	    t = Math::clamp(t, 0, a);
	    //ClipToRange(t,0,a);
	  }

	  u = t*A_dot_B - B_dot_T;
	  if (u < 0)
	  {
	    u = 0;
	    t = A_dot_T;
	    t = Math::clamp(t, 0, a);
	    //ClipToRange(t,0,a);
	  }
	  else if (u > b)
	  {
	    u = b;
	    t = u*A_dot_B + A_dot_T;
	    t = Math::clamp(t, 0, a);
	    //ClipToRange(t,0,a);
	  }
	}

    template<class T>
    double RSSDistanceCalc<T>::distance(
        const rw::geometry::OBB<T>& obbA,
        const rw::geometry::OBB<T>& obbB,
        const rw::math::Transform3D<T>& aTb)
    {
      using namespace rw::math;

      T t, s;

      const Vector3D<T> &a = obbA.getHalfLengths();
      const Vector3D<T> &b = obbB.getHalfLengths();

      // aTbf = fabs(aTb.R)
      const Vector3D<T>& aPb = aTb.P();
      const Rotation3D<T> &aRb = aTb.R();

      const Rotation3D<T> &bRa = inverse(aRb);
      const Vector3D<T>& bPa = -(bRa*aPb);

      // this is the rotated a*A_x+ vector
      T a_x0 = a[0]*aRb(0,0);
      T a_x1 = a[0]*aRb(0,1);
      // this is the rotated a*A_y+ vector
      T a_y0 = a[1]*aRb(1,0);
      T a_y1 = a[1]*aRb(1,1);

      // this is the rotated b*B_x+ vector
      T b_x0 = b[0]*aRb(0,0);
      T b_x1 = b[0]*aRb(1,0);

      // this is the rotated b*b_y+ vector
      T b_y0 = b[1]*aRb(0,1);
      T b_y1 = b[1]*aRb(1,1);

      // we define the edges of A and B as the segments
      // A_x+, A_e1: {( a(0), a(1)), ( a(0),-a(1))}
      // A_x-, A_e2: {(-a(0), a(1)), (-a(0),-a(1))}
      // A_y+, A_e3: {( a(0), a(1)), (-a(0), a(1))}
      // A_y-, A_e4: {( a(0),-a(1)), (-a(0),-a(1))}

      // the end points of a line segment on B is denoted pBU_x (upper) and pBL_x (lower)
      // which means that the pBU_x is the segment with a larger x value than pBL_x

      // determine if any edge pair contain closest points.

      // the first test is to check if an edge is in the exterior halfspace of a



      // test A_x+, B_x+

      // we first determine if the point is outside the exterier halfspace
      // if the largest cb_x value is below the a*A_x+ then it is outside
/*
      T S[3], t, u;

      T pBU_x = -b_y0;
      T pBL_x =  b_x0;
      if( b_x0>-b_y0 ){
          pU_x =  b_x0;
          pL_x = -b_y0;
      }

      T pAU_x = -a_y0;
      T pAL_x =  a_x0;
      if( a_x0>-a_y0 ){
          pAU_x =  a_x0;
          pAL_x = -a_y0;
      }



      if( (a[0]< aPb[0] + pBU_x) && ( b[0]<bPa[0] + pAU_x)){
          // the segments A_x+,B_x+ is either inside the exterior halfspace or in-and-outside
          // if the segments are entirely inside then
          if( (a[0] < aPb[0] + pBL_x) && (b[0] < bPa[0] + pAL_x) ){
              // segments are completely inside
              // calculate the closest points between two line segments
              SegCoords(t,u,a[1],b[1],A1_dot_B1,Tab[1] + bA1_dot_B0,
                        Tba[1] - aA0_dot_B1);


              return DistanceUtil::distanceLineLine( );
          }
          RW_THROW("10");
      }
      RW_THROW("17");
*/
      return 0;
   }

}
}

#endif
