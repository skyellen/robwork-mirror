#ifndef RW_PROXIMITY_OBBTOLERANCECOLLIDER_HPP_
#define RW_PROXIMITY_OBBTOLERANCECOLLIDER_HPP_

#include <rw/geometry/OBB.hpp>
#include <rw/geometry/BVCollider.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw {
namespace geometry {

    /**
     * @brief class for testing if two Oriented Bounding Boxes are overlapping
     */
    template<class T=double>
    class OBBToleranceCollider : public BVCollider<OBBToleranceCollider<T>, rw::geometry::OBB<T> >{
    public:
        typedef typename rw::geometry::OBB<T> BVType;
        typedef T value_type;

        //! @brief constructor
        OBBToleranceCollider(){};

        //! @brief destructor
        virtual ~OBBToleranceCollider(){};

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
        bool collides(const rw::geometry::OBB<T>& obbA,
                      const rw::geometry::OBB<T>& obbB,
                      const rw::math::Transform3D<T>& aTb){
            return collides(obbA,obbB,aTb,_tolerance);
        }

        /**
         * @brief test if obbA is closer than \b tolerance to obbB. The aTb transform describe
         * obbB relative to obbA's coordinate frame. This method is approximate and the obb's
         * may lie up to sqrt(tolerance^2+tolerance^2) away from each other.
         */
        bool collides(const rw::geometry::OBB<T>& obbA,
                      const rw::geometry::OBB<T>& obbB,
                      const rw::math::Transform3D<T>& aTb,
                      double tolerance);

    private:
        T _tolerance;

    };

    template<class T>
    bool OBBToleranceCollider<T>::collides(
        const rw::geometry::OBB<T>& obbA, const rw::geometry::OBB<T>& obbB, const rw::math::Transform3D<T>& aTb, double tolerance)
    {
        using namespace rw::math;

        T t, s;

        Vector3D<T> a = obbA.getHalfLengths();
        a[0] += tolerance;
        a[1] += tolerance;
        a[2] += tolerance;
        const Vector3D<T> &b = obbB.getHalfLengths();
        //std::cout << a << b << std::endl;
        //std::cout << aTb << std::endl;
        // aTbf = fabs(aTb.R)
        const Vector3D<T>& aPb = aTb.P();
        const Rotation3D<T> &aRb = aTb.R();
        //Rotation3D<T> aRbf;
        T aRbf[3][3];
        const T reps = (T)1e-6;

        aRbf[0][0] = fabs(aRb(0,0));  aRbf[0][0] += reps;
        aRbf[0][1] = fabs(aRb(0,1));  aRbf[0][1] += reps;
        aRbf[0][2] = fabs(aRb(0,2));  aRbf[0][2] += reps;
        aRbf[1][0] = fabs(aRb(1,0));  aRbf[1][0] += reps;
        aRbf[1][1] = fabs(aRb(1,1));  aRbf[1][1] += reps;
        aRbf[1][2] = fabs(aRb(1,2));  aRbf[1][2] += reps;
        aRbf[2][0] = fabs(aRb(2,0));  aRbf[2][0] += reps;
        aRbf[2][1] = fabs(aRb(2,1));  aRbf[2][1] += reps;
        aRbf[2][2] = fabs(aRb(2,2));  aRbf[2][2] += reps;

        //
        int r = 1;

        // A1 x A2 = A0
        t = fabs(aPb[0]);

        r &= (t <=
            (a[0] + b[0] * aRbf[0][0] + b[1] * aRbf[0][1] + b[2] * aRbf[0][2]));
        if (!r)
            return false; // 1

        // B1 x B2 = B0
        s = aPb[0]*aRb(0,0) + aPb[1]*aRb(1,0) + aPb[2]*aRb(2,0);
        t = fabs(s);

        r &= ( t <=
            (b[0] + a[0] * aRbf[0][0] + a[1] * aRbf[1][0] + a[2] * aRbf[2][0]));
        if (!r) return false; // (2)

        // A2 x A0 = A1
        t = fabs(aPb[1]);

        r &= ( t <=
            (a[1] + b[0] * aRbf[1][0] + b[1] * aRbf[1][1] + b[2] * aRbf[1][2]));
        if (!r) return false; // 3;

        // A0 x A1 = A2
        t = fabs(aPb[2]);

        r &= ( t <=
            (a[2] + b[0] * aRbf[2][0] + b[1] * aRbf[2][1] + b[2] * aRbf[2][2]));
        if (!r) return false; // 4;

        // B2 x B0 = B1
        s = aPb[0]*aRb(0,1) + aPb[1]*aRb(1,1) + aPb[2]*aRb(2,1);
        t = fabs(s);

        r &= ( t <=
            (b[1] + a[0] * aRbf[0][1] + a[1] * aRbf[1][1] + a[2] * aRbf[2][1]));
        if (!r) return false; // 5;

        // B0 x B1 = B2
        s = aPb[0]*aRb(0,2) + aPb[1]*aRb(1,2) + aPb[2]*aRb(2,2);
        t = fabs(s);

        r &= ( t <=
            (b[2] + a[0] * aRbf[0][2] + a[1] * aRbf[1][2] + a[2] * aRbf[2][2]));
        if (!r) return false; // 6;

        // A0 x B0
        s = aPb[2] * aRb(1,0) - aPb[1] * aRb(2,0);
        t = fabs(s);

        r &= ( t <=
          (a[1] * aRbf[2][0] + a[2] * aRbf[1][0] +
           b[1] * aRbf[0][2] + b[2] * aRbf[0][1]));
        if (!r) return false; // 7;

        // A0 x B1
        s = aPb[2] * aRb(1,1) - aPb[1] * aRb(2,1);
        t = fabs(s);

        r &= ( t <=
          (a[1] * aRbf[2][1] + a[2] * aRbf[1][1] +
           b[0] * aRbf[0][2] + b[2] * aRbf[0][0]));
        if (!r) return false; // 8;

        // A0 x B2
        s = aPb[2] * aRb(1,2) - aPb[1] * aRb(2,2);
        t = fabs(s);

        r &= ( t <=
            (a[1] * aRbf[2][2] + a[2] * aRbf[1][2] +
             b[0] * aRbf[0][1] + b[1] * aRbf[0][0]));
        if (!r) return false; // 9;

        // A1 x B0
        s = aPb[0] * aRb(2,0) - aPb[2] * aRb(0,0);
        t = fabs(s);

        r &= ( t <=
            (a[0] * aRbf[2][0] + a[2] * aRbf[0][0] +
             b[1] * aRbf[1][2] + b[2] * aRbf[1][1]));
        if (!r) return false;

        // A1 x B1
        s = aPb[0] * aRb(2,1) - aPb[2] * aRb(0,1);
        t = fabs(s);

        r &= ( t <=
            (a[0] * aRbf[2][1] + a[2] * aRbf[0][1] +
             b[0] * aRbf[1][2] + b[2] * aRbf[1][0]));
        if (!r) return false; // 11;

        // A1 x B2
        s = aPb[0] * aRb(2,2) - aPb[2] * aRb(0,2);
        t = fabs(s);

        r &= (t <=
            (a[0] * aRbf[2][2] + a[2] * aRbf[0][2] +
             b[0] * aRbf[1][1] + b[1] * aRbf[1][0]));
        if (!r) return false; // 12;

        // A2 x B0
        s = aPb[1] * aRb(0,0) - aPb[0] * aRb(1,0);
        t = fabs(s);

        r &= (t <=
            (a[0] * aRbf[1][0] + a[1] * aRbf[0][0] +
             b[1] * aRbf[2][2] + b[2] * aRbf[2][1]));
        if (!r) return false; // 13;

        // A2 x B1
        s = aPb[1] * aRb(0,1) - aPb[0] * aRb(1,1);
        t = fabs(s);

        r &= ( t <=
            (a[0] * aRbf[1][1] + a[1] * aRbf[0][1] +
             b[0] * aRbf[2][2] + b[2] * aRbf[2][0]));
        if (!r) return false; // 14;

        // A2 x B2
        s = aPb[1] * aRb(0,2) - aPb[0] * aRb(1,2);
        t = fabs(s);

        r &= ( t <=
            (a[0] * aRbf[1][2] + a[1] * aRbf[0][2] +
             b[0] * aRbf[2][1] + b[1] * aRbf[2][0]));
        if (!r) return false; // 15;

        return true;  // should equal 0
    }
}
}

#endif /* BVCOLLIDER_HPP_ */
