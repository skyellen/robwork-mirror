
#ifndef RW_GEOMETRY_TRITRITOLERANCEINTERSECT_HPP_
#define RW_GEOMETRY_TRITRITOLERANCEINTERSECT_HPP_

#include <rw/geometry/Triangle.hpp>
#include <rw/math/Transform3D.hpp>
#include "TriDistanceCalc.hpp"
#include "BVCollider.hpp"

namespace rw {
namespace geometry {

    /**
     * @brief tests if two swept sphere triangles are intersecting. This corresponds
     * to testing if two triangles are within a certain distqance (tolerance) of each
     * other
     */
    template<class T=double>
    class TriTriToleranceIntersect: public BVCollider<TriTriToleranceIntersect<T>, rw::geometry::Triangle<T> > {
    public:
    	//! @brief Type for internal values.
        typedef T value_type;

        /**
         * @brief set the tolerance of the collider
         * @param tolerance [in] meters
         */
        void setTolerance(T tolerance){ _tolerance = tolerance;}

        /**
         * @brief test if obbA is closer than \b tolerance to obbB. The aTb transform describe
         * obbB relative to obbA's coordinate frame. This method is approximate and the obb's
         * may lie up to sqrt(tolerance^2+tolerance^2) away from each other.
         */
        bool collides(const rw::geometry::Triangle<T>& a,
                      const rw::geometry::Triangle<T>& b,
                      const rw::math::Transform3D<T>& aTb){
            return collides(a,b,aTb,_tolerance);
        }

        /**
         * @brief test if obbA is closer than \b tolerance to obbB. The aTb transform describe
         * obbB relative to obbA's coordinate frame. This method is approximate and the obb's
         * may lie up to sqrt(tolerance^2+tolerance^2) away from each other.
         */
        bool collides(const rw::geometry::Triangle<T>& a,
                      const rw::geometry::Triangle<T>& b,
                      const rw::math::Transform3D<T>& aTb,
                      double tolerance)
        {
            T dist = _tritridist.distance(a,b,aTb);
            if(dist<tolerance)
                return true;
            return false;
        }

    private:
        T _tolerance;
        TriDistanceCalc<T> _tritridist;
    };



}
}

#endif /* TRIDEVILLER_HPP_ */
