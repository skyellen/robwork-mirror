/*
 * BVCollider.hpp
 *
 *  Created on: 28-10-2008
 *      Author: jimali
 */

#ifndef RW_PROXIMITY_RSSDISTANCECALC_HPP_
#define RW_PROXIMITY_RSSDISTANCECALC_HPP_

#include <sandbox/geometry/OBB.hpp>
#include "BVDistanceCalc.hpp"
#include <rw/math/Vector3D.hpp>
#include "DistanceUtil.hpp"
namespace rw {
namespace proximity {

	/**
	 * @brief class for testing if two Oriented Bounding Boxes are overlapping
	 */

	template<class T=double>
	class RSSDistanceCalc : public BVDistanceCalc<RSSDistanceCalc<T>, rw::geometry::OBB<T> > {
	public:
		typedef typename rw::geometry::OBB<> BVType;
		typedef T value_type;

		//! @brief constructor
		RSSDistanceCalc(){};

		//! @brief destructor
		virtual ~RSSDistanceCalc(){};

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

	  std::cout << "aPb: " << aPb << std::endl;
	  std::cout << "bPa: " << bPa << std::endl;

	  // the vectors describing the b halflength in A's reference frame
	  const Vector3D<T>& b_x = aRb*Vector3D<T>(b(0),0,0);
	  const Vector3D<T>& b_y = aRb*Vector3D<T>(0,b(1),0);

	  // the vectors describing the a halflength in B's reference frame
      const Vector3D<T>& a_x = bRa*Vector3D<T>(a(0),0,0);
      const Vector3D<T>& a_y = bRa*Vector3D<T>(0,a(1),0);

      const double AxDotBx = aRb(0,0);
      const double AxDotBy = aRb(0,1);
      const double AyDotBx = aRb(1,0);
      const double AyDotBy = aRb(1,1);

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
	      std::cout << "1";
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
          std::cout << "2";
          // find closest points on segments
          return DistanceUtil::distanceLineLine(
                  Vector3D<>(-a(0),a(1),0), Vector3D<>(-a(0),-a(1),0),
                  aPb+b_x+b_y, aPb+b_x-b_y );
      }

      // is B_e1 inside halfspace of A_e3, and vice verse
      bInsideA = (a(1) < aPb(1)+b_x(1)+b_y(1)) && (a(1) < aPb(1)+b_x(1)-b_y(1));
      AInsideB = (b(0) < bPa(0)+a_x(0)+a_y(0)) && (b(0) < bPa(0)+a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){
          std::cout << "3";
          return DistanceUtil::distanceLineLine(
                  Vector3D<>(a(0),a(1),0), Vector3D<>(-a(0),a(1),0),
                  aPb+b_x+b_y, aPb+b_x-b_y );
          // find closest points on segments
      }

      // is B_e1 inside halfspace of A_e4, and vice verse
      bInsideA = (-a(1) > aPb(1)+b_x(1)+b_y(1)) && (-a(1) > aPb(1)+b_x(1)-b_y(1));
      AInsideB = ( b(0) < bPa(0)+a_x(0)-a_y(0)) && ( b(0) < bPa(0)-a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){
          std::cout << "4";
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
          std::cout << "5";
          // find closest points on segments
      }

      // is B_e2 inside halfspace of A_e2, and vice verse
      bInsideA = (-a(0) > aPb(0)-b_x(0)+b_y(0)) && (-a(0) > aPb(0)-b_x(0)-b_y(0));
      AInsideB = (-b(0) > bPa(0)-a_x(0)+a_y(0)) && (-b(0) > bPa(0)-a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){
          std::cout << "6";
          std::cout << -b(0)<< ">" << bPa(0)<<"-" <<a_x(0)<<"+"<<a_y(0) << "&&" << -b(0) << ">" << bPa(0)-a_x(0)-a_y(0) << std::endl;
          std::cout << Vector3D<>(-a(0),a(1),0)
                    << Vector3D<>(-a(0),-a(1),0) << "\n"
                    << aPb-b_x+b_y << aPb-b_x-b_y << "\n";

          return DistanceUtil::distanceLineLine(
                  Vector3D<>(-a(0),a(1),0), Vector3D<>(-a(0),-a(1),0),
                  aPb-b_x+b_y, aPb-b_x-b_y );
          // find closest points on segments
      }

      // is B_e2 inside halfspace of A_e3, and vice verse
      bInsideA = ( a(1) < aPb(1)-b_x(1)+b_y(1)) && ( a(1) < aPb(1)-b_x(1)-b_y(1));
      AInsideB = (-b(0) > bPa(0)+a_x(0)+a_y(0)) && (-b(0) > bPa(0)+a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){
          std::cout << "7";
          // find closest points on segments
      }

      // is B_e2 inside halfspace of A_e4, and vice verse
      bInsideA = (-a(1) > aPb(1)-b_x(1)+b_y(1)) && (-a(1) > aPb(1)-b_x(1)-b_y(1));
      AInsideB = (-b(0) > bPa(0)+a_x(0)-a_y(0)) && (-b(0) > bPa(0)-a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){
          std::cout << "8";
          // find closest points on segments
      }




      // now B_e3, +-x,+y
      // is B_e3 inside halfspace of A_e1, and vice verse
      bInsideA = (a(0) < aPb(0)+b_x(0)+b_y(0)) && (a(0) < aPb(0)-b_x(0)+b_y(0));
      AInsideB = (b(1) < bPa(1)+a_x(0)+a_y(1)) && (b(1) < bPa(1)+a_x(1)-a_y(1));
      if( bInsideA && AInsideB ){
          std::cout << "9";
          // find closest points on segments
      }

      // is B_e3 inside halfspace of A_e2, and vice verse
      bInsideA = (-a(0) > aPb(0)+b_x(0)+b_y(0)) && (-a(0) > aPb(0)-b_x(0)+b_y(0));
      AInsideB = ( b(1) < bPa(1)-a_x(1)+a_y(1)) && ( b(1) < bPa(0)-a_x(1)-a_y(1));
      if( bInsideA && AInsideB ){
          std::cout << "10";
          // find closest points on segments
      }

      // is B_e3 inside halfspace of A_e3, and vice verse
      bInsideA = (a(1) < aPb(1)+b_x(1)+b_y(1)) && (a(1) < aPb(1)-b_x(1)+b_y(1));
      AInsideB = (b(1) < bPa(1)+a_x(1)+a_y(1)) && (b(1) < bPa(1)+a_x(1)-a_y(1));
      if( bInsideA && AInsideB ){
          std::cout << "11";
          // find closest points on segments
      }

      // is B_e1 inside halfspace of A_e4, and vice verse
      bInsideA = (-a(1) > aPb(1)+b_x(1)+b_y(1)) && (-a(1) > aPb(1)-b_x(1)+b_y(1));
      AInsideB = ( b(1) < bPa(1)+a_x(0)-a_y(1)) && ( b(1) < bPa(1)-a_x(1)-a_y(1));
      if( bInsideA && AInsideB ){
          std::cout << "12";
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
          std::cout << "13";
          // find closest points on segments
      }

      // is B_e4 inside halfspace of A_e2, and vice verse
      bInsideA = (-a(0) > aPb(0)+b_x(0)-b_y(0)) && (-a(0) > aPb(0)-b_x(0)-b_y(0));
      AInsideB = (-b(0) > bPa(0)-a_x(0)+a_y(0)) && (-b(0) > bPa(0)-a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){
          std::cout << "14";
          // find closest points on segments
      }

      // is B_e4 inside halfspace of A_e3, and vice verse
      bInsideA = ( a(1) < aPb(1)+b_x(1)-b_y(1)) && ( a(1) < aPb(1)-b_x(1)-b_y(1));
      AInsideB = (-b(0) > bPa(0)+a_x(0)+a_y(0)) && (-b(0) > bPa(0)+a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){
          std::cout << "15";
          // find closest points on segments
      }

      // is B_e4 inside halfspace of A_e4, and vice verse
      bInsideA = (-a(1) > aPb(1)+b_x(1)-b_y(1)) && (-a(1) > aPb(1)-b_x(1)-b_y(1));
      AInsideB = (-b(0) > bPa(0)+a_x(0)-a_y(0)) && (-b(0) > bPa(0)-a_x(0)-a_y(0));
      if( bInsideA && AInsideB ){
          std::cout << "16";
          // find closest points on segments
      }



      // we compute minimum distance between all possible segment pairs and return the minimum of these



	  // and we compute point rectangle plane distances


	  return 0;  // should equal 0
	}



}
}

#endif
